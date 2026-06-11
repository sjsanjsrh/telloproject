from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

from camera_calibration.camera_tranceform import load_camera_params
from yolo_seg.flight_plan import flight_plan_points, load_flight_plan, nearest_path_point
from yolo_seg.pnp_pos_est import SquarePoseDetection, YoloSquarePoseEstimator
from yolo_seg.pnp_pos_est_viz import (
	LatestFrameSlamPump,
	LatestFrameYoloWorker,
	_resolve_inference_model,
	build_horizontal_dashboard,
	build_pose_markers,
	estimate_camera_fov_deg,
	first_obstacle,
	load_optional_scene_map,
	parse_vector3,
	partial_gate_cue_from_projection,
	resolve_obstacle,
	select_pose_detection,
)
from yolo_seg.pose_fusion import CommandPose, PoseFusion
from yolo_seg.scene_3d_viz import create_scene_3d_visualizer
from yolo_seg.slam_backend import SlamPose, create_slam_backend
from yolo_seg.world_pose import CameraWorldPose


@dataclass
class PoseWorkerResult:
	timestamp: float
	fused_pose: Optional[CameraWorldPose]
	vision_pose: Optional[CameraWorldPose]
	slam_pose: Optional[SlamPose]
	command_pose: Optional[CameraWorldPose]
	selected_detection: Optional[SquarePoseDetection]
	pose_confidence: float
	status: str

	@property
	def best_pose(self) -> Optional[CameraWorldPose]:
		return self.fused_pose or self.vision_pose or self.command_pose


class TelloPoseWorker:
	"""Run front-camera YOLO/PnP/SLAM fusion without blocking flight commands."""

	def __init__(self, tello, args):
		self.tello = tello
		self.args = args
		self._stop_event = threading.Event()
		self._ready_event = threading.Event()
		self._failed_event = threading.Event()
		self._thread: Optional[threading.Thread] = None
		self._lock = threading.Lock()
		self._command_lock = threading.Lock()
		self._latest: Optional[PoseWorkerResult] = None
		self._startup_error: Optional[str] = None
		self._command_pose: Optional[CommandPose] = None

	def start(self) -> None:
		if self._thread is not None:
			return
		self._thread = threading.Thread(target=self._run, name="tello-pose-worker", daemon=True)
		self._thread.start()

	def stop(self, timeout: float = 2.0) -> None:
		self._stop_event.set()
		if self._thread is not None:
			self._thread.join(timeout=timeout)
			self._thread = None

	def wait_ready(self, timeout: Optional[float] = None) -> bool:
		deadline = None if timeout is None else time.time() + float(timeout)
		while True:
			if self._ready_event.is_set():
				return True
			if self._failed_event.is_set():
				return False
			if deadline is not None and time.time() >= deadline:
				return False
			time.sleep(0.05)

	@property
	def startup_error(self) -> Optional[str]:
		return self._startup_error

	def set_command_pose(
		self,
		position_cm,
		rotation_matrix,
		status: str = "command ack",
		segment_index: int = 0,
	) -> None:
		command_pose = CommandPose(
			camera_world_pose=CameraWorldPose(
				position_cm=tuple(float(value) for value in np.asarray(position_cm, dtype=np.float64).reshape(3)),
				rotation_matrix=np.asarray(rotation_matrix, dtype=np.float64).reshape(3, 3),
				method=status,
			),
			status=status,
			segment_index=int(segment_index),
		)
		with self._command_lock:
			self._command_pose = command_pose

	def current_command_pose(self) -> Optional[CommandPose]:
		with self._command_lock:
			return self._command_pose

	def latest(self, max_age_sec: Optional[float] = None) -> Optional[PoseWorkerResult]:
		with self._lock:
			result = self._latest
		if result is None:
			return None
		if max_age_sec is not None and time.time() - result.timestamp > max_age_sec:
			return None
		return result

	def _set_latest(self, result: PoseWorkerResult) -> None:
		with self._lock:
			self._latest = result

	def _run(self) -> None:
		slam_backend = None
		scene_3d = None
		yolo_worker = None
		slam_pump = None
		try:
			args = self.args
			scene_map = load_optional_scene_map(args.scene_map)
			flight_plan = load_flight_plan(args.flight_plan)
			flight_points = flight_plan_points(flight_plan)
			config_obstacle = first_obstacle(scene_map)
			object_position_cm = parse_vector3(getattr(args, "object_position_cm", None))
			object_yaw_deg = float(getattr(args, "object_yaw_deg", 0.0))
			if config_obstacle is not None and object_position_cm is None:
				object_position_cm = config_obstacle.position_cm
				object_yaw_deg = config_obstacle.yaw_deg
			target_width_cm = getattr(args, "target_width_cm", None) or (config_obstacle.shape.width_cm if config_obstacle is not None else None)
			target_height_cm = getattr(args, "target_height_cm", None) or (config_obstacle.shape.height_cm if config_obstacle is not None else None)
			outline_thickness_cm = getattr(args, "outline_thickness_cm", None) or (
				config_obstacle.shape.thickness_cm if config_obstacle is not None and config_obstacle.shape.thickness_cm > 0.0 else None
			)
			square_size_cm = getattr(args, "square_size_cm", None) or target_width_cm or target_height_cm
			if square_size_cm is None:
				raise ValueError("Target size is missing. Set yolo_seg/obstacles.yaml shape.size_cm or square_size_cm.")

			camera_profile = getattr(args, "effective_camera_profile", None) or getattr(args, "camera_profile", None)
			camera_direction = getattr(args, "effective_camera_direction", None) or getattr(args, "camera", None)
			resolution = getattr(args, "effective_resolution", None) or getattr(args, "resolution", None)
			camera_matrix, dist_coeffs = load_camera_params(
				args.camera_params,
				camera_profile=camera_profile,
				camera_direction=camera_direction,
				resolution=resolution,
			)
			hfov_deg, vfov_deg = estimate_camera_fov_deg(camera_matrix)
			estimator = YoloSquarePoseEstimator(
				camera_matrix=camera_matrix,
				dist_coeffs=dist_coeffs,
				square_size_cm=square_size_cm,
				target_width_cm=target_width_cm,
				target_height_cm=target_height_cm,
				target_class_id=getattr(args, "target_class_id", None),
				target_class_name=getattr(args, "target_class_name", None),
				outline_class_id=getattr(args, "outline_class_id", None),
				outline_class_name=getattr(args, "outline_class_name", None),
				outline_thickness_cm=outline_thickness_cm,
				min_area_px=args.min_area_px,
			)
			model, inference_device = _resolve_inference_model(args.pose_model, args.device, args.imgsz)
			slam_backend = create_slam_backend(args)
			slam_backend.start()
			yolo_worker = LatestFrameYoloWorker(model, estimator, args.imgsz, args.conf, inference_device, float(getattr(args, "yolo_fps", 0.0)))
			slam_pump = LatestFrameSlamPump(
				slam_backend,
				float(getattr(args, "slam_fps", 0.0)),
				input_mode=getattr(args, "slam_input", "bgr"),
			)
			yolo_worker.start()
			slam_pump.start()
			fusion = PoseFusion()
			if not args.no_3d:
				scene_3d = create_scene_3d_visualizer(
					camera_fov_deg=(hfov_deg, vfov_deg),
					prefer_gpu=args.scene_3d_renderer == "gpu",
				)
			self._ready_event.set()
		except Exception as exc:
			self._startup_error = str(exc)
			self._failed_event.set()
			print(f"pose worker startup error: {exc}")
			return

		last_camera_world_pose: Optional[CameraWorldPose] = None
		last_selected_detection: Optional[SquarePoseDetection] = None
		last_vision_pose: Optional[CameraWorldPose] = None
		last_yolo_result = None
		last_yolo_frame = None
		last_detections = []
		last_yolo_sequence = 0
		last_yolo_fps = 0.0
		last_inference_ms = 0.0
		slam_pose = None
		slam_status = "off"
		while not self._stop_event.is_set():
			frame = self.tello.get_frame()
			if frame is None:
				time.sleep(0.01)
				continue

			now = time.time()
			yolo_worker.submit(frame)
			slam_pump.submit(frame)
			new_slam_pose = slam_backend.poll()
			if new_slam_pose is not None:
				slam_pose = new_slam_pose
			slam_status = slam_backend.status_text()
			command = self.current_command_pose() if args.command_prior else None
			command_pose = command.camera_world_pose if command is not None else None

			vision_pose = None
			selected_detection = last_selected_detection
			detections = last_detections
			yolo_output = yolo_worker.poll()
			new_yolo_measurement = yolo_output is not None and int(yolo_output["sequence"]) != last_yolo_sequence
			if new_yolo_measurement:
				last_yolo_sequence = int(yolo_output["sequence"])
				last_yolo_frame = yolo_output["frame"]
				last_yolo_result = yolo_output["result"]
				last_inference_ms = float(yolo_output["inference_ms"])
				last_yolo_fps = float(yolo_output["fps"])

				detections = estimator.estimate_all_from_result(last_yolo_result, reference_detection=last_selected_detection) if last_yolo_result is not None else []
				selected_detection, _obstacle, vision_pose = select_pose_detection(
					detections=detections,
					scene_map=scene_map,
					object_id=getattr(args, "object_id", None),
					config_obstacle=config_obstacle,
					object_position_cm=object_position_cm,
					object_yaw_deg=object_yaw_deg,
					command_pose=command_pose,
					last_camera_world_pose=last_camera_world_pose,
					slam_pose=slam_pose,
					camera_matrix=camera_matrix,
					dist_coeffs=dist_coeffs,
				)
				last_detections = detections
				last_selected_detection = selected_detection
				if vision_pose is not None:
					last_vision_pose = vision_pose
			pose_confidence = selected_detection.pose_confidence if selected_detection is not None else 0.0
			disable_pnp_fusion = getattr(args, "disable_pnp_fusion", False)
			fusion_pnp_pose = None if getattr(args, "disable_pnp_fusion", False) else vision_pose
			fusion_pnp_confidence = 0.0 if getattr(args, "disable_pnp_fusion", False) or not new_yolo_measurement else pose_confidence
			fused = fusion.update(
				slam_pose=slam_pose,
				pnp_pose=fusion_pnp_pose,
				pnp_confidence=fusion_pnp_confidence,
				command_pose=command_pose,
				command_noise_cm=args.command_noise_cm,
				flight_path_points=flight_points if args.command_prior else None,
				timestamp=now,
			)
			fused_pose = fused.camera_world_pose if fused is not None else None
			if vision_pose is not None and not disable_pnp_fusion:
				last_camera_world_pose = vision_pose
			elif fused_pose is not None:
				last_camera_world_pose = fused_pose

			status = fused.status if fused is not None else slam_status
			pnp_applied = fused is not None and any(part in {"pnp", "pnp init"} for part in fused.status.split("+"))
			partial_gate_obstacle = config_obstacle or resolve_obstacle(scene_map, getattr(args, "object_id", None), None)
			partial_gate_cue = partial_gate_cue_from_projection(
				detections=detections,
				obstacle=partial_gate_obstacle,
				camera_pose=command_pose or fused_pose or (slam_pose.camera_world_pose if slam_pose is not None else None),
				camera_matrix=camera_matrix,
				dist_coeffs=dist_coeffs,
			)
			self._set_latest(
				PoseWorkerResult(
					timestamp=now,
					fused_pose=fused_pose,
					vision_pose=vision_pose or last_vision_pose,
					slam_pose=slam_pose,
					command_pose=command_pose,
					selected_detection=selected_detection,
					pose_confidence=pose_confidence,
					status=status,
				)
			)

			path_projection = nearest_path_point(flight_points, fused_pose.position_cm) if fused_pose is not None else None
			visible_camera_pose = (
				fused_pose
				or (slam_pose.camera_world_pose if slam_pose is not None else None)
				or command_pose
				or (None if disable_pnp_fusion else vision_pose)
			)
			if scene_3d is not None:
				scene_3d.show(
					scene_map=scene_map,
					camera_pose=visible_camera_pose,
					flight_path_points=flight_points,
					pose_markers=build_pose_markers(
						vision_pose,
						slam_pose,
						fused_pose,
						command_pose,
						path_projection,
						pose_confidence=pose_confidence,
						slam_status=slam_status,
					),
				)
			if not args.no_pose_viz:
				dashboard = build_horizontal_dashboard(
					frame,
					last_yolo_result,
					selected_detection,
					detections,
					estimator,
					Path(args.pose_model).name,
					panel_height=args.panel_height,
					read_ms=0.0,
					inference_ms=last_inference_ms,
					postprocess_ms=0.0,
					fps=0.0,
					object_position_cm=object_position_cm,
					object_yaw_deg=object_yaw_deg,
					slam_status=f"{slam_status} {status} yolo={last_yolo_fps:.1f} slam={slam_pump.fps():.1f}",
					yolo_fps=last_yolo_fps,
					slam_fps=slam_pump.fps(),
					vision_frame=last_yolo_frame,
					fusion_status=status,
					pnp_applied=pnp_applied,
					partial_gate_cue=partial_gate_cue,
				)
				cv2.imshow(args.pose_window_name, dashboard)
				cv2.waitKey(1)

		if yolo_worker is not None:
			yolo_worker.stop()
		if slam_pump is not None:
			slam_pump.stop()
		if slam_backend is not None:
			slam_backend.close()
		if not args.no_pose_viz:
			cv2.destroyWindow(args.pose_window_name)
