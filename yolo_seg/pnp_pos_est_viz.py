"""Horizontal visualization for square pose estimation with YOLO segmentation.

The script loads a YOLO segmentation model, estimates a square pose with
``YoloSquarePoseEstimator``, and shows a landscape-oriented dashboard with the
original frame, YOLO overlay, pose-annotated frame, and a text summary panel.
The segmentation overlay is outline-only so ring-shaped targets keep their hole
visible instead of appearing filled.

Usage:

	python yolo_seg/pnp_pos_est_viz_vertical.py --source 0
	python yolo_seg/pnp_pos_est_viz_vertical.py --source sample.mp4 --model yolo26n-seg.pt
"""

from __future__ import annotations

import argparse
import sys
import shutil
import time
from time import perf_counter
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import torch
from ultralytics import YOLO

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
	sys.path.insert(0, str(PROJECT_ROOT))

from camera_calibration.camera_tranceform import load_camera_params
from telloController import TelloController
from yolo_seg.flight_plan import flight_plan_points, load_flight_plan, nearest_path_point
from yolo_seg.pnp_pos_est import SquarePoseDetection, YoloSquarePoseEstimator
from yolo_seg.scene_map import Obstacle, SceneMap, load_scene_map
from yolo_seg.scene_3d_viz import create_scene_3d_visualizer
from yolo_seg.slam_backend import SlamPose, create_slam_backend
from yolo_seg.world_pose import CameraWorldPose, estimate_camera_world_pose


DEFAULT_MODEL = "yolo_seg/res/runs/segment/train/weights/best.pt"
DEFAULT_CAMERA_PARAMS = Path("camera_calibration") / "camera_params.yaml"
DEFAULT_SCENE_MAP = Path("yolo_seg") / "obstacles.yaml"
DEFAULT_FLIGHT_PLAN = Path("flight_path.yaml")
DEFAULT_SQUARE_SIZE_CM = 20.0


def parse_args() -> argparse.Namespace:
	parser = argparse.ArgumentParser(description="Horizontal pose visualization for YOLO segmentation results")
	parser.add_argument("--source", default="tello", help="Video source path, webcam index, or 'tello'")
	parser.add_argument("--model", default=DEFAULT_MODEL, help="YOLO segmentation model path")
	parser.add_argument("--camera-params", default=str(DEFAULT_CAMERA_PARAMS), help="Camera calibration YAML path")
	parser.add_argument("--camera-profile", default=None, help="Calibration profile name, e.g. downward or forward_480p")
	parser.add_argument("--camera", default=None, help="Camera profile camera: forward/front or downward/bottom")
	parser.add_argument("--resolution", type=int, default=None, choices=(480, 720), help="Camera profile resolution")
	parser.add_argument("--square-size-cm", type=float, default=DEFAULT_SQUARE_SIZE_CM, help="Square target size in cm")
	parser.add_argument("--target-class-id", type=int, default=None, help="Target class id to track")
	parser.add_argument("--target-class-name", default=None, help="Target class name to track")
	parser.add_argument("--outline-class-id", type=int, default=None, help="Partial outline class id for rough thickness-based pose")
	parser.add_argument("--outline-class-name", default=None, help="Partial outline class name for rough thickness-based pose")
	parser.add_argument("--outline-thickness-cm", type=float, default=None, help="Real outline stroke thickness in cm")
	parser.add_argument("--target-width-cm", type=float, default=None, help="Target outer width in cm")
	parser.add_argument("--target-height-cm", type=float, default=None, help="Target outer height in cm")
	parser.add_argument("--scene-map", default=str(DEFAULT_SCENE_MAP), help="Scene obstacle YAML path")
	parser.add_argument("--flight-plan", default=str(DEFAULT_FLIGHT_PLAN), help="Flight path YAML path for 3D visualization")
	parser.add_argument("--object-id", default=None, help="Known obstacle id from --scene-map, e.g. A, B, or C")
	parser.add_argument("--object-position-cm", default=None, help="Known target Tello/world position as forward,right,up in cm")
	parser.add_argument("--object-yaw-deg", type=float, default=0.0, help="Known target yaw around Tello/world up axis")
	parser.add_argument("--camera-direction", default="forward", choices=("forward", "downward"), help="Tello camera direction when using --source tello")
	parser.add_argument("--video-resolution", type=int, default=480, choices=(480, 720), help="Tello video resolution when using --source tello")
	parser.add_argument("--min-area-px", type=float, default=200.0, help="Minimum contour area in pixels")
	parser.add_argument("--imgsz", type=int, default=640, help="Inference image size")
	parser.add_argument("--conf", type=float, default=0.25, help="Confidence threshold")
	parser.add_argument("--device", default="auto", help="Inference device, e.g. auto, intel:NPU, cpu")
	parser.add_argument("--panel-height", type=int, default=360, help="Output panel height")
	parser.add_argument("--window-name", default="pnp_pos_est_horizontal", help="OpenCV window name")
	parser.add_argument("--no-3d", action="store_true", help="Disable the separate 3D scene window")
	parser.add_argument("--3d-renderer", dest="scene_3d_renderer", default="gpu", choices=("gpu", "cpu"), help="3D scene renderer backend")
	parser.add_argument("--slam-backend", default="none", choices=("none", "orbslam3_py"), help="SLAM pose source")
	parser.add_argument("--slam-scale-cm", type=float, default=100.0, help="Scale for SLAM position records without position_cm, e.g. meters to cm")
	parser.add_argument("--orbslam3-vocab", default=None, help="ORB-SLAM3 vocabulary file, usually ORBvoc.txt")
	parser.add_argument("--orbslam3-settings", default=None, help="ORB-SLAM3 camera/settings YAML")
	parser.add_argument("--orbslam3-py-module", default=None, help="Path to orbslam3_py .pyd module for in-process SLAM")
	return parser.parse_args()


def _normalize_device_name(device_name: str) -> str:
	device_name = device_name.strip()
	if not device_name:
		return "auto"
	return device_name


def _openvino_model_dir(model_path: str | Path) -> Path:
	model_path = Path(model_path)
	return model_path.parent / f"{model_path.stem}_openvino_model"


def _ensure_openvino_model(model_path: str | Path, imgsz: int) -> Path:
	"""Export a PyTorch YOLO model to OpenVINO if needed and return the model directory."""

	model_path = Path(model_path)
	openvino_dir = _openvino_model_dir(model_path)
	if openvino_dir.exists() and any(openvino_dir.glob("*.xml")):
		return openvino_dir

	if openvino_dir.exists():
		shutil.rmtree(openvino_dir)

	model = YOLO(str(model_path))
	exported = model.export(format="openvino", imgsz=imgsz, verbose=False)
	return Path(exported) if exported is not None else openvino_dir


def _resolve_inference_model(model_path: str, device: str, imgsz: int) -> tuple[YOLO, str]:
	"""Resolve the runtime model and device for CPU, CUDA, or Intel NPU."""

	device = _normalize_device_name(device)
	if device == "auto":
		if torch.cuda.is_available():
			device = "cuda:0"
		else:
			try:
				import openvino as ov

				available_devices = ov.Core().available_devices
			except Exception:
				available_devices = []

			if "NPU" in available_devices:
				device = "intel:NPU"
			else:
				device = "cpu"

	if device.lower().startswith("intel:npu"):
		openvino_dir = _ensure_openvino_model(model_path, imgsz)
		return YOLO(str(openvino_dir)), "intel:NPU"

	if device.lower().startswith("cuda"):
		return YOLO(model_path), "cuda:0"

	if device.lower() == "cpu":
		return YOLO(model_path), "cpu"

	return YOLO(model_path), device


def parse_source(source: str):
	if source.strip().lower() == "tello":
		return "tello"
	try:
		return int(source)
	except ValueError:
		return source


def parse_vector3(value: Optional[str]) -> Optional[tuple[float, float, float]]:
	if value is None:
		return None
	parts = [part.strip() for part in value.split(",")]
	if len(parts) != 3:
		raise ValueError("--object-position-cm must look like forward,right,up")
	return float(parts[0]), float(parts[1]), float(parts[2])


def estimate_camera_fov_deg(camera_matrix: np.ndarray) -> tuple[float, float]:
	fx = float(camera_matrix[0, 0])
	fy = float(camera_matrix[1, 1])
	cx = float(camera_matrix[0, 2])
	cy = float(camera_matrix[1, 2])
	image_width = max(1.0, cx * 2.0)
	image_height = max(1.0, cy * 2.0)
	hfov_deg = float(np.degrees(2.0 * np.arctan(image_width / (2.0 * fx))))
	vfov_deg = float(np.degrees(2.0 * np.arctan(image_height / (2.0 * fy))))
	return hfov_deg, vfov_deg


def build_pose_markers(
	vision_pose: Optional[CameraWorldPose],
	slam_pose: Optional[SlamPose],
	path_projection: Optional[dict],
	pose_confidence: float = 0.0,
	slam_status: Optional[str] = None,
) -> list[dict]:
	markers: list[dict] = []
	if path_projection is not None:
		markers.append(
			{
				"label": "path",
				"position_cm": path_projection["position_cm"],
				"color_bgr": (255, 220, 80),
			}
		)
	if slam_pose is not None:
		markers.append(
			{
				"label": "orb",
				"position_cm": slam_pose.position_cm,
				"color_bgr": (255, 180, 60),
			}
		)
	elif slam_status and "off" not in slam_status:
		label = "orb WAIT"
		if "inproc error:" in slam_status:
			label = "orb ERR"
		elif "inproc frames 0" in slam_status:
			label = "orb INIT"
		elif "inproc frames" in slam_status:
			label = "orb"
		markers.append(
			{
				"label": label,
				"position_cm": (0.0, 0.0, 0.0),
				"color_bgr": (80, 160, 255),
			}
		)
	if vision_pose is not None:
		markers.append(
			{
				"label": "vision",
				"position_cm": vision_pose.position_cm,
				"color_bgr": (0, 255, 120),
			}
		)
	return markers


def load_optional_scene_map(path: Optional[str]) -> Optional[SceneMap]:
	if not path:
		return None
	scene_path = Path(path)
	if not scene_path.exists():
		return None
	return load_scene_map(scene_path)


def first_obstacle(scene_map: Optional[SceneMap]) -> Optional[Obstacle]:
	if scene_map is None or not scene_map.obstacles:
		return None
	skip_types = {"start_area", "start_rectangle", "landing_point", "landing_zone", "landing_marker", "circle_pillar"}
	for obstacle in scene_map.obstacles.values():
		if obstacle.shape.type.lower() not in skip_types:
			return obstacle
	return next(iter(scene_map.obstacles.values()))


def resolve_obstacle(
	scene_map: Optional[SceneMap],
	object_id: Optional[str],
	detection: Optional[SquarePoseDetection] = None,
) -> Optional[Obstacle]:
	if scene_map is None:
		return None
	if object_id:
		return scene_map.get(object_id)
	if detection is None:
		return None
	return scene_map.match_detection(detection.class_id, detection.class_name)


class TelloStreamCapture:
	def __init__(self, camera_direction: str, video_resolution: int):
		self.tello = TelloController()
		self.closed = False
		self.camera_direction = TelloController.CAMERA_FORWARD if camera_direction == "forward" else TelloController.CAMERA_DOWNWARD
		self.video_resolution = video_resolution

		self.tello.start(motor_on=False)
		self.tello.set_video_bitrate(self.tello.BITRATE_1MBPS)
		self.tello.set_video_fps(self.tello.FPS_30)
		self.tello.set_video_resolution(self.tello.RESOLUTION_480P if video_resolution == 480 else self.tello.RESOLUTION_720P)
		self.tello.setUpVideo(show_video=False, camera_direction=self.camera_direction)
		self.tello.printInfo()

		while not self.tello.can_read_frame():
			time.sleep(0.05)

	def read(self):
		while not self.closed:
			frame = self.tello.get_frame()
			if frame is not None:
				return True, frame
			time.sleep(0.01)
		return False, None

	def release(self):
		self.closed = True
		try:
			self.tello.closseVideo()
		except Exception:
			pass


def resize_to_width(image: np.ndarray, width: int) -> np.ndarray:
	height, current_width = image.shape[:2]
	if current_width == width:
		return image
	scale = width / float(current_width)
	new_height = max(1, int(round(height * scale)))
	return cv2.resize(image, (width, new_height), interpolation=cv2.INTER_AREA)


def resize_to_height(image: np.ndarray, height: int) -> np.ndarray:
	current_height, width = image.shape[:2]
	if current_height == height:
		return image
	scale = height / float(current_height)
	new_width = max(1, int(round(width * scale)))
	return cv2.resize(image, (new_width, height), interpolation=cv2.INTER_AREA)


def resize_to_fit_height(image: np.ndarray, height: int) -> np.ndarray:
	return resize_to_height(image, height)


def make_text_panel(width: int, lines: list[str]) -> np.ndarray:
	height = 36 + max(0, len(lines) - 1) * 28 + 28
	panel = np.zeros((height, width, 3), dtype=np.uint8)
	panel[:] = (18, 18, 18)

	for index, line in enumerate(lines):
		y = 36 + index * 28
		cv2.putText(panel, line, (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.68, (245, 245, 245), 2, cv2.LINE_AA)

	return panel


def add_detection_info(frame: np.ndarray, detection: Optional[SquarePoseDetection]) -> np.ndarray:
	output = frame.copy()
	if detection is None:
		cv2.putText(output, "no detection", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
		return output

	center = tuple(int(round(value)) for value in detection.center_px)
	corners = detection.corners_px.astype(np.int32).reshape(-1, 1, 2)
	cv2.polylines(output, [corners], True, (0, 255, 0), 2)
	cv2.circle(output, center, 4, (0, 0, 255), -1)

	label = f"{detection.pose_method} pose={detection.pose_confidence:.2f} area={detection.area_px:.0f}px conf={detection.confidence:.2f}"
	if detection.class_name:
		label = f"{detection.class_name} {label}"
	cv2.putText(output, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2, cv2.LINE_AA)

	return output


def build_outline_overlay(frame: np.ndarray, result) -> np.ndarray:
	output = frame.copy()
	masks = getattr(result, "masks", None)
	boxes = getattr(result, "boxes", None)
	names = getattr(result, "names", None)

	if masks is None:
		return output

	mask_polygons = getattr(masks, "xy", None)
	if mask_polygons is not None:
		for index, polygon in enumerate(mask_polygons):
			contour = np.asarray(polygon, dtype=np.int32).reshape(-1, 1, 2)
			if len(contour) < 3:
				continue

			color = (0, 255, 0)
			class_name = None
			if boxes is not None and len(boxes) > index:
				class_tensor = boxes.cls[index]
				class_id = int(class_tensor.item() if hasattr(class_tensor, "item") else class_tensor)
				if isinstance(names, dict):
					class_name = names.get(class_id)
				color = ((37 * (class_id + 1)) % 255, (91 * (class_id + 1)) % 255, (149 * (class_id + 1)) % 255)

			cv2.fillPoly(output, [contour], color)

			if class_name and boxes is not None and len(boxes) > index:
				box = boxes.xyxy[index]
				if hasattr(box, "tolist"):
					box = box.tolist()
				x1, y1, _x2, _y2 = map(int, box)
				cv2.putText(output, class_name, (x1, max(20, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)
		return output

	if getattr(masks, "data", None) is None:
		return output

	mask_data = masks.data
	if hasattr(mask_data, "detach"):
		mask_array = mask_data.detach().cpu().numpy()
	else:
		mask_array = np.asarray(mask_data)

	if mask_array.ndim == 2:
		mask_array = mask_array[np.newaxis, ...]

	for index, mask in enumerate(mask_array):
		binary_mask = (mask > 0.5).astype(np.uint8) * 255
		if binary_mask.shape[:2] != frame.shape[:2]:
			binary_mask = cv2.resize(binary_mask, (frame.shape[1], frame.shape[0]), interpolation=cv2.INTER_NEAREST)

		contours, _hierarchy = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		if not contours:
			continue

		color = (0, 255, 0)
		class_name = None
		if boxes is not None and len(boxes) > index:
			class_tensor = boxes.cls[index]
			class_id = int(class_tensor.item() if hasattr(class_tensor, "item") else class_tensor)
			if isinstance(names, dict):
				class_name = names.get(class_id)
			if class_id is not None:
				color = ((37 * (class_id + 1)) % 255, (91 * (class_id + 1)) % 255, (149 * (class_id + 1)) % 255)

		cv2.drawContours(output, contours, -1, color, thickness=-1)

		if class_name and boxes is not None and len(boxes) > index:
			box = boxes.xyxy[index]
			if hasattr(box, "tolist"):
				box = box.tolist()
			x1, y1, x2, y2 = map(int, box)
			cv2.putText(output, class_name, (x1, max(20, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)

	return output


def build_horizontal_dashboard(
	frame: np.ndarray,
	result,
	detection: Optional[SquarePoseDetection],
	estimator: YoloSquarePoseEstimator,
	model_name: str,
	panel_height: int,
	read_ms: float,
	inference_ms: float,
	postprocess_ms: float,
	fps: float,
	object_position_cm: Optional[tuple[float, float, float]],
	object_yaw_deg: float,
	slam_status: Optional[str] = None,
) -> np.ndarray:
	original_panel = resize_to_height(frame, panel_height)
	plot_panel = resize_to_fit_height(build_outline_overlay(frame, result), panel_height) if result is not None else original_panel.copy()
	pose_panel = resize_to_height(add_detection_info(frame, detection), panel_height)

	if detection is None:
		status_lines = [
			f"model: {model_name}",
			"status: no valid square detection",
			f"inference: {inference_ms:.1f} ms",
			f"fps: {fps:.1f}",
		]
	else:
		status_lines = [
			f"model: {model_name}",
			f"target: {detection.class_name or detection.class_id or 'square'}",
			f"method: {detection.pose_method}",
			f"inference: {inference_ms:.1f} ms",
			f"fps: {fps:.1f}",
			f"score: {detection.score:.1f}   conf: {detection.confidence:.2f}   pose: {detection.pose_confidence:.2f}",
		]
	if slam_status:
		status_lines.append(f"slam: {slam_status}")

	image_strip = np.hstack([original_panel, plot_panel, pose_panel])
	info_panel = make_text_panel(image_strip.shape[1], status_lines)
	return np.vstack([image_strip, info_panel])


def open_capture(source, camera_direction: str, video_resolution: int):
	if source == "tello":
		return TelloStreamCapture(camera_direction=camera_direction, video_resolution=video_resolution)

	capture = cv2.VideoCapture(source)
	if not capture.isOpened():
		raise RuntimeError(f"Failed to open source: {source}")
	return capture


def main() -> None:
	args = parse_args()
	source = parse_source(args.source)
	scene_map = load_optional_scene_map(args.scene_map)
	flight_path_points = flight_plan_points(load_flight_plan(args.flight_plan))
	config_obstacle = resolve_obstacle(scene_map, args.object_id) or first_obstacle(scene_map)
	object_position_cm = parse_vector3(args.object_position_cm)
	object_yaw_deg = args.object_yaw_deg
	if object_position_cm is None and config_obstacle is not None:
		object_position_cm = config_obstacle.position_cm
		object_yaw_deg = config_obstacle.yaw_deg
	calibration_camera = args.camera
	calibration_resolution = args.resolution
	if source == "tello":
		calibration_camera = calibration_camera or args.camera_direction
		calibration_resolution = calibration_resolution or args.video_resolution

	camera_matrix, dist_coeffs = load_camera_params(
		args.camera_params,
		camera_profile=args.camera_profile,
		camera_direction=calibration_camera,
		resolution=calibration_resolution,
	)
	estimator = YoloSquarePoseEstimator(
		camera_matrix=camera_matrix,
		dist_coeffs=dist_coeffs,
		square_size_cm=args.square_size_cm,
		target_width_cm=args.target_width_cm or (config_obstacle.shape.width_cm if config_obstacle is not None else None),
		target_height_cm=args.target_height_cm or (config_obstacle.shape.height_cm if config_obstacle is not None else None),
		target_class_id=args.target_class_id,
		target_class_name=args.target_class_name,
		outline_class_id=args.outline_class_id,
		outline_class_name=args.outline_class_name,
		outline_thickness_cm=args.outline_thickness_cm or (config_obstacle.shape.thickness_cm if config_obstacle is not None else None),
		min_area_px=args.min_area_px,
	)

	model, inference_device = _resolve_inference_model(args.model, args.device, args.imgsz)
	capture = open_capture(source, args.camera_direction, args.video_resolution)
	slam_backend = create_slam_backend(args)
	camera_fov_deg = estimate_camera_fov_deg(camera_matrix)
	scene_3d = (
		create_scene_3d_visualizer(prefer_gpu=args.scene_3d_renderer == "gpu", camera_fov_deg=camera_fov_deg)
		if (scene_map is not None or flight_path_points) and not args.no_3d
		else None
	)
	fps_ema = 0.0
	last_postprocess_ms = 0.0
	last_camera_world_pose = None
	last_camera_local_detection_cm = None
	slam_pose = None
	path_projection = None

	try:
		slam_backend.start()
		while True:
			frame_start = perf_counter()
			read_start = perf_counter()
			ok, frame = capture.read()
			read_ms = (perf_counter() - read_start) * 1000.0
			if not ok or frame is None:
				break
			if hasattr(slam_backend, "push_frame"):
				slam_backend.push_frame(frame)

			inference_start = perf_counter()
			results = model.predict(frame, imgsz=args.imgsz, conf=args.conf, device=inference_device, verbose=False)
			inference_ms = (perf_counter() - inference_start) * 1000.0
			result = results[0] if results else None

			postprocess_start = perf_counter()
			new_slam_pose = slam_backend.poll()
			if new_slam_pose is not None:
				slam_pose = new_slam_pose
				path_projection = nearest_path_point(flight_path_points, slam_pose.position_cm)
			detection = estimator.estimate_from_result(result) if result is not None else None
			detection_obstacle = resolve_obstacle(scene_map, args.object_id, detection)
			reference_obstacle = detection_obstacle or config_obstacle
			dashboard_object_position_cm = object_position_cm
			dashboard_object_yaw_deg = object_yaw_deg
			if args.object_position_cm is None and reference_obstacle is not None:
				dashboard_object_position_cm = reference_obstacle.position_cm
				dashboard_object_yaw_deg = reference_obstacle.yaw_deg
			camera_world_pose = None
			if detection is not None and dashboard_object_position_cm is not None:
				camera_world_pose = estimate_camera_world_pose(
					detection,
					dashboard_object_position_cm,
					dashboard_object_yaw_deg,
				)
			if camera_world_pose is not None:
				last_camera_world_pose = camera_world_pose
				if slam_pose is None:
					path_projection = nearest_path_point(flight_path_points, camera_world_pose.position_cm)
			if detection is not None and detection.position_cm is not None:
				last_camera_local_detection_cm = detection.position_cm
			dashboard = build_horizontal_dashboard(
				frame=frame,
				result=result,
				detection=detection,
				estimator=estimator,
				model_name=str(args.model),
				panel_height=args.panel_height,
				read_ms=read_ms,
				inference_ms=inference_ms,
				postprocess_ms=last_postprocess_ms,
				fps=fps_ema,
				object_position_cm=dashboard_object_position_cm,
				object_yaw_deg=dashboard_object_yaw_deg,
				slam_status=slam_backend.status_text(),
			)
			last_postprocess_ms = (perf_counter() - postprocess_start) * 1000.0

			frame_end = perf_counter()
			frame_seconds = frame_end - frame_start
			if frame_seconds > 0:
				instant_fps = 1.0 / frame_seconds
				fps_ema = instant_fps if fps_ema == 0.0 else (fps_ema * 0.9 + instant_fps * 0.1)
			else:
				fps_ema = 0.0

			cv2.imshow(args.window_name, dashboard)
			if scene_3d is not None:
				visible_camera_pose = (
					slam_pose.camera_world_pose
					if slam_pose is not None
					else camera_world_pose if camera_world_pose is not None else last_camera_world_pose
				)
				scene_3d.show(
					scene_map,
					active_obstacle=detection_obstacle,
					camera_pose=visible_camera_pose,
					camera_local_detection_cm=(
						detection.position_cm
						if detection is not None and detection.position_cm is not None
						else last_camera_local_detection_cm
					),
					flight_path_points=flight_path_points,
					pose_markers=build_pose_markers(
						visible_camera_pose,
						slam_pose,
						path_projection,
						pose_confidence=detection.pose_confidence if detection is not None else 0.0,
						slam_status=slam_backend.status_text(),
					),
				)
			key = cv2.waitKey(1) & 0xFF
			if scene_3d is not None and hasattr(scene_3d, "handle_key"):
				scene_3d.handle_key(key)
			if key in (27, ord("q")):
				break
	finally:
		capture.release()
		slam_backend.close()
		cv2.destroyAllWindows()


if __name__ == "__main__":
	main()
