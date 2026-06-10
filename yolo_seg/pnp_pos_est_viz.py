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
import hashlib
import json
import sys
import shutil
import threading
import time
import xml.etree.ElementTree as ET
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
from yolo_seg.pose_fusion import CommandPoseTracker, PoseFusion
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
	parser.add_argument("--yolo-fps", type=float, default=0.0, help="Maximum YOLO inference FPS; 0 means unlimited/latest frame")
	parser.add_argument("--slam-fps", type=float, default=0.0, help="Maximum SLAM input FPS; 0 means unlimited/latest frame")
	parser.add_argument("--device", default="auto", help="Inference device, e.g. auto, intel:NPU, cpu")
	parser.add_argument("--panel-height", type=int, default=360, help="Output panel height")
	parser.add_argument("--window-name", default="pnp_pos_est_horizontal", help="OpenCV window name")
	parser.add_argument("--no-3d", action="store_true", help="Disable the separate 3D scene window")
	parser.add_argument("--3d-renderer", dest="scene_3d_renderer", default="gpu", choices=("gpu", "cpu"), help="3D scene renderer backend")
	parser.add_argument("--slam-only-viz", action="store_true", help="In 3D view, show only the converted SLAM camera pose, not PnP/fused pose")
	parser.add_argument("--command-prior", action="store_true", help="Use flight path as the currently commanded trajectory prior")
	parser.add_argument("--command-speed-cm-s", type=float, default=50.0, help="Command trajectory speed when --command-prior is enabled")
	parser.add_argument("--command-noise-cm", type=float, default=45.0, help="Kalman measurement noise for command prior")
	parser.add_argument("--command-start-index", type=int, default=0, help="Flight path segment index to start command prior from")
	parser.add_argument("--slam-backend", default="none", choices=("none", "orbslam3_py"), help="SLAM pose source")
	parser.add_argument("--slam-scale-cm", type=float, default=100.0, help="Scale for SLAM position records without position_cm, e.g. meters to cm")
	parser.add_argument("--slam-sync", action="store_true", help="Run SLAM track() in the visualization loop instead of the SLAM worker thread")
	parser.add_argument("--slam-input", default="bgr", choices=("bgr", "gray"), help="Frame format passed to ORB-SLAM3")
	parser.add_argument("--orbslam3-vocab", default=None, help="ORB-SLAM3 vocabulary file, usually ORBvoc.txt")
	parser.add_argument("--orbslam3-settings", default=None, help="ORB-SLAM3 camera/settings YAML")
	parser.add_argument("--orbslam3-py-module", default=None, help="Path to orbslam3_py .pyd module for in-process SLAM")
	return parser.parse_args()


def _update_fps_ema(previous: float, last_time: Optional[float], current_time: float) -> tuple[float, float]:
	if last_time is None:
		return previous, current_time
	delta = current_time - last_time
	if delta <= 0.0:
		return previous, current_time
	instant_fps = 1.0 / delta
	return (instant_fps if previous == 0.0 else previous * 0.9 + instant_fps * 0.1), current_time


def preprocess_slam_frame(frame: np.ndarray, input_mode: str) -> np.ndarray:
	if input_mode == "gray":
		return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
	return frame


def slam_frame_stats(frame: Optional[np.ndarray]) -> str:
	if frame is None:
		return "slam input none"
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
	mean = float(np.mean(gray))
	std = float(np.std(gray))
	lap = float(cv2.Laplacian(gray, cv2.CV_64F).var())
	height, width = gray.shape[:2]
	channels = 1 if frame.ndim == 2 else frame.shape[2]
	return f"input {width}x{height}x{channels} mean={mean:.1f} std={std:.1f} lap={lap:.1f}"


class LatestFrameYoloWorker:
	def __init__(
		self,
		model: YOLO,
		estimator: YoloSquarePoseEstimator,
		imgsz: int,
		conf: float,
		device: str,
		max_fps: float,
	):
		self.model = model
		self.estimator = estimator
		self.imgsz = int(imgsz)
		self.conf = float(conf)
		self.device = device
		self.period = (1.0 / max_fps) if max_fps > 0.0 else 0.0
		self._lock = threading.Lock()
		self._stop_event = threading.Event()
		self._thread = threading.Thread(target=self._run, name="pnp-yolo-worker", daemon=True)
		self._pending_frame = None
		self._latest_output = None
		self._last_run_at = None
		self._fps_ema = 0.0
		self._sequence = 0

	def start(self) -> None:
		self._thread.start()

	def submit(self, frame: np.ndarray) -> None:
		with self._lock:
			if self._pending_frame is not None:
				return
			self._pending_frame = frame.copy()

	def poll(self):
		with self._lock:
			return self._latest_output

	def stop(self) -> bool:
		self._stop_event.set()
		self._thread.join(timeout=2.0)
		return not self._thread.is_alive()

	def _take_frame(self):
		with self._lock:
			frame = self._pending_frame
			self._pending_frame = None
			return frame

	def _publish(self, output) -> None:
		with self._lock:
			self._latest_output = output

	def _run(self) -> None:
		while not self._stop_event.is_set():
			now = perf_counter()
			if self._last_run_at is not None and self.period > 0.0:
				sleep_seconds = self.period - (now - self._last_run_at)
				if sleep_seconds > 0.0:
					self._stop_event.wait(min(sleep_seconds, 0.01))
					continue

			frame = self._take_frame()
			if frame is None:
				self._stop_event.wait(0.002)
				continue

			try:
				run_started_at = perf_counter()
				self._fps_ema, self._last_run_at = _update_fps_ema(self._fps_ema, self._last_run_at, run_started_at)
				inference_start = perf_counter()
				results = self.model.predict(frame, imgsz=self.imgsz, conf=self.conf, device=self.device, verbose=False)
				inference_ms = (perf_counter() - inference_start) * 1000.0
				result = results[0] if results else None
				detections = self.estimator.estimate_all_from_result(result) if result is not None else []
				self._sequence += 1
				self._publish(
					{
						"sequence": self._sequence,
						"frame": frame,
						"result": result,
						"detections": detections,
						"inference_ms": inference_ms,
						"fps": self._fps_ema,
						"error": None,
					}
				)
			except Exception as exc:
				self._sequence += 1
				self._publish(
					{
						"sequence": self._sequence,
						"frame": frame,
						"result": None,
						"detections": [],
						"inference_ms": 0.0,
						"fps": self._fps_ema,
						"error": str(exc),
					}
				)


class LatestFrameSlamPump:
	def __init__(self, slam_backend, max_fps: float, input_mode: str = "bgr"):
		self.slam_backend = slam_backend
		self.period = (1.0 / max_fps) if max_fps > 0.0 else 0.0
		self.input_mode = input_mode
		self._lock = threading.Lock()
		self._stop_event = threading.Event()
		self._thread = threading.Thread(target=self._run, name="pnp-slam-pump", daemon=True)
		self._pending_frame = None
		self._last_push_at = None
		self._fps_ema = 0.0
		self._submitted = 0
		self._accepted = 0
		self._dropped = 0
		self._processed = 0
		self._last_input_stats = "slam input none"

	def start(self) -> None:
		self._thread.start()

	def submit(self, frame: np.ndarray) -> None:
		with self._lock:
			self._submitted += 1
			if self._pending_frame is not None:
				self._dropped += 1
				return
			self._accepted += 1
			self._pending_frame = frame.copy()

	def fps(self) -> float:
		with self._lock:
			return self._fps_ema

	def stats_text(self) -> str:
		with self._lock:
			pending = 1 if self._pending_frame is not None else 0
			return f"slam q={pending} in={self._accepted}/{self._submitted} drop={self._dropped} proc={self._processed} {self._last_input_stats}"

	def is_alive(self) -> bool:
		return self._thread.is_alive()

	def stop(self) -> bool:
		self._stop_event.set()
		self._thread.join(timeout=2.0)
		return not self._thread.is_alive()

	def _take_frame(self):
		with self._lock:
			frame = self._pending_frame
			self._pending_frame = None
			return frame

	def _set_fps(self, fps_ema: float) -> None:
		with self._lock:
			self._fps_ema = fps_ema

	def _run(self) -> None:
		while not self._stop_event.is_set():
			now = perf_counter()
			if self._last_push_at is not None and self.period > 0.0:
				sleep_seconds = self.period - (now - self._last_push_at)
				if sleep_seconds > 0.0:
					self._stop_event.wait(min(sleep_seconds, 0.01))
					continue

			frame = self._take_frame()
			if frame is None:
				self._stop_event.wait(0.002)
				continue

			if hasattr(self.slam_backend, "push_frame"):
				push_started_at = perf_counter()
				new_fps_ema, self._last_push_at = _update_fps_ema(self._fps_ema, self._last_push_at, push_started_at)
				slam_frame = preprocess_slam_frame(frame, self.input_mode)
				input_stats = slam_frame_stats(slam_frame)
				self.slam_backend.push_frame(slam_frame)
				with self._lock:
					self._processed += 1
					self._fps_ema = new_fps_ema
					self._last_input_stats = input_stats


def _normalize_device_name(device_name: str) -> str:
	device_name = device_name.strip()
	if not device_name:
		return "auto"
	return device_name


def _openvino_model_dir(model_path: str | Path) -> Path:
	model_path = Path(model_path)
	return model_path.parent / f"{model_path.stem}_openvino_model"


def _read_openvino_input_size(openvino_dir: str | Path) -> Optional[int]:
	xml_files = sorted(Path(openvino_dir).glob("*.xml"))
	if not xml_files:
		return None

	root = ET.parse(xml_files[0]).getroot()
	for layer in root.iter():
		if layer.attrib.get("type") != "Parameter":
			continue

		shape = layer.attrib.get("shape") or layer.findtext("data/shape")
		if shape:
			dims = [
				int(value)
				for value in shape.replace("[", "").replace("]", "").replace(",", " ").split()
				if value.strip().lstrip("-").isdigit()
			]
			if len(dims) >= 4 and dims[-1] == dims[-2]:
				return int(dims[-1])

		dims = []
		for dim in layer.iter():
			if dim.tag.endswith("dim") and dim.text and dim.text.strip().lstrip("-").isdigit():
				dims.append(int(dim.text.strip()))
		if len(dims) >= 4 and dims[-1] == dims[-2]:
			return int(dims[-1])

	return None


def _file_sha256(path: str | Path) -> str:
	digest = hashlib.sha256()
	with open(path, "rb") as file_handle:
		for chunk in iter(lambda: file_handle.read(1024 * 1024), b""):
			digest.update(chunk)
	return digest.hexdigest()


def _openvino_manifest_path(openvino_dir: str | Path) -> Path:
	return Path(openvino_dir) / "telloproject_export_manifest.json"


def _read_openvino_manifest(openvino_dir: str | Path) -> dict:
	manifest_path = _openvino_manifest_path(openvino_dir)
	if not manifest_path.exists():
		return {}
	try:
		with open(manifest_path, "r", encoding="utf-8") as file_handle:
			return json.load(file_handle) or {}
	except (OSError, json.JSONDecodeError):
		return {}


def _write_openvino_manifest(openvino_dir: str | Path, model_path: str | Path, imgsz: int) -> None:
	manifest = {
		"source_model": str(Path(model_path)),
		"source_sha256": _file_sha256(model_path),
		"imgsz": int(imgsz),
		"export_format": "openvino",
	}
	with open(_openvino_manifest_path(openvino_dir), "w", encoding="utf-8") as file_handle:
		json.dump(manifest, file_handle, indent=2, sort_keys=True)


def _ensure_openvino_model(model_path: str | Path, imgsz: int) -> Path:
	"""Export a PyTorch YOLO model to OpenVINO if needed and return the model directory."""

	model_path = Path(model_path)
	openvino_dir = _openvino_model_dir(model_path)
	source_hash = _file_sha256(model_path)
	if openvino_dir.exists() and any(openvino_dir.glob("*.xml")):
		existing_imgsz = _read_openvino_input_size(openvino_dir)
		manifest = _read_openvino_manifest(openvino_dir)
		hash_matches = manifest.get("source_sha256") == source_hash
		imgsz_matches = int(manifest.get("imgsz", -1)) == int(imgsz)
		if existing_imgsz == int(imgsz) and hash_matches and imgsz_matches:
			return openvino_dir
		reasons = []
		if existing_imgsz != int(imgsz):
			reasons.append(f"shape existing={existing_imgsz}, requested={imgsz}")
		if not hash_matches:
			reasons.append("source model hash changed or missing")
		if not imgsz_matches:
			reasons.append("export manifest imgsz changed or missing")
		print(f"OpenVINO model mismatch: {', '.join(reasons)}. Re-exporting.")
		shutil.rmtree(openvino_dir)

	if openvino_dir.exists():
		shutil.rmtree(openvino_dir)

	model = YOLO(str(model_path))
	exported = model.export(format="openvino", imgsz=imgsz, verbose=False)
	exported_dir = Path(exported) if exported is not None else openvino_dir
	_write_openvino_manifest(exported_dir, model_path, imgsz)
	return exported_dir


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
	fused_pose: Optional[CameraWorldPose],
	command_pose: Optional[CameraWorldPose],
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
	if fused_pose is not None:
		markers.append(
			{
				"label": "fused",
				"position_cm": fused_pose.position_cm,
				"color_bgr": (245, 245, 245),
			}
		)
	if command_pose is not None:
		markers.append(
			{
				"label": "cmd",
				"position_cm": command_pose.position_cm,
				"color_bgr": (70, 210, 255),
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


def select_pose_detection(
	detections: list[SquarePoseDetection],
	scene_map: Optional[SceneMap],
	object_id: Optional[str],
	config_obstacle: Optional[Obstacle],
	object_position_cm: Optional[tuple[float, float, float]],
	object_yaw_deg: float,
	command_pose: Optional[CameraWorldPose],
	last_camera_world_pose: Optional[CameraWorldPose],
	slam_pose: Optional[SlamPose],
) -> tuple[Optional[SquarePoseDetection], Optional[Obstacle], Optional[CameraWorldPose]]:
	best: tuple[float, Optional[SquarePoseDetection], Optional[Obstacle], Optional[CameraWorldPose]] = (-float("inf"), None, None, None)
	for detection in detections:
		if detection.avoidance_only or detection.tvec is None:
			continue

		obstacle = resolve_obstacle(scene_map, object_id, detection) or config_obstacle
		target_position = obstacle.position_cm if obstacle is not None else object_position_cm
		target_yaw = obstacle.yaw_deg if obstacle is not None else object_yaw_deg
		if target_position is None:
			continue

		camera_world_pose = estimate_camera_world_pose(detection, target_position, target_yaw)
		if camera_world_pose is None:
			continue

		position = np.asarray(camera_world_pose.position_cm, dtype=np.float64)
		score = detection.pose_confidence * 1000.0 + detection.confidence * 100.0
		if command_pose is not None:
			score -= 0.035 * float(np.linalg.norm(position - np.asarray(command_pose.position_cm, dtype=np.float64)))
		if last_camera_world_pose is not None:
			score -= 0.025 * float(np.linalg.norm(position - np.asarray(last_camera_world_pose.position_cm, dtype=np.float64)))
		if slam_pose is not None:
			score -= 0.01 * float(np.linalg.norm(position - np.asarray(slam_pose.position_cm, dtype=np.float64)))
		if detection.pose_method == "outline_thickness":
			score -= 120.0

		if score > best[0]:
			best = (score, detection, obstacle, camera_world_pose)

	return best[1], best[2], best[3]


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


def add_detection_info(
	frame: np.ndarray,
	detection: Optional[SquarePoseDetection],
	detections: Optional[list[SquarePoseDetection]] = None,
) -> np.ndarray:
	output = frame.copy()
	all_detections = detections or ([detection] if detection is not None else [])
	if not all_detections:
		cv2.putText(output, "no detection", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
		return output

	for index, item in enumerate(all_detections):
		center = tuple(int(round(value)) for value in item.center_px)
		corners = item.corners_px.astype(np.int32).reshape(-1, 1, 2)
		color = (0, 180, 255) if item.avoidance_only else (0, 255, 0)
		thickness = 3 if item is detection else 1
		cv2.polylines(output, [corners], True, color, thickness)
		cv2.circle(output, center, 4, (0, 0, 255), -1)
		label = f"{index}:{item.class_name or item.class_id or 'gate'} {item.pose_method}"
		cv2.putText(output, label, (center[0] + 6, center[1] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)

	if detection is None:
		cv2.putText(output, "avoidance only", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 180, 255), 2, cv2.LINE_AA)
		return output

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
	detections: Optional[list[SquarePoseDetection]],
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
	yolo_fps: float = 0.0,
	slam_fps: float = 0.0,
	vision_frame: Optional[np.ndarray] = None,
) -> np.ndarray:
	vision_frame = frame if vision_frame is None else vision_frame
	original_panel = resize_to_height(frame, panel_height)
	plot_panel = resize_to_fit_height(build_outline_overlay(vision_frame, result), panel_height) if result is not None else resize_to_height(vision_frame, panel_height)
	pose_panel = resize_to_height(add_detection_info(vision_frame, detection, detections), panel_height)

	if detection is None:
		status_lines = [
			f"model: {model_name}",
			"status: no valid square detection",
			f"candidates: {len(detections or [])}",
			f"loop fps: {fps:.1f}   read: {read_ms:.1f} ms",
			f"yolo fps: {yolo_fps:.1f}   infer: {inference_ms:.1f} ms",
			f"slam fps: {slam_fps:.1f}   post: {postprocess_ms:.1f} ms",
		]
	else:
		status_lines = [
			f"model: {model_name}",
			f"target: {detection.class_name or detection.class_id or 'square'}",
			f"method: {detection.pose_method}",
			f"candidates: {len(detections or [])}   corners: {detection.corner_count}",
			f"loop fps: {fps:.1f}   read: {read_ms:.1f} ms",
			f"yolo fps: {yolo_fps:.1f}   infer: {inference_ms:.1f} ms",
			f"slam fps: {slam_fps:.1f}   post: {postprocess_ms:.1f} ms",
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


def resolve_effective_camera_calibration(args, source) -> tuple[Optional[str], Optional[int]]:
	camera_direction = args.camera
	resolution = args.resolution
	if source == "tello":
		camera_direction = camera_direction or args.camera_direction
		resolution = resolution or args.video_resolution

	args.effective_camera_profile = args.camera_profile
	args.effective_camera_direction = camera_direction
	args.effective_resolution = resolution
	return camera_direction, resolution


def main() -> None:
	args = parse_args()
	source = parse_source(args.source)
	scene_map = load_optional_scene_map(args.scene_map)
	flight_path_points = flight_plan_points(load_flight_plan(args.flight_plan))
	command_tracker = (
		CommandPoseTracker(
			flight_path_points,
			speed_cm_s=args.command_speed_cm_s,
			start_index=args.command_start_index,
		)
		if args.command_prior and flight_path_points
		else None
	)
	config_obstacle = resolve_obstacle(scene_map, args.object_id) or first_obstacle(scene_map)
	object_position_cm = parse_vector3(args.object_position_cm)
	object_yaw_deg = args.object_yaw_deg
	if object_position_cm is None and config_obstacle is not None:
		object_position_cm = config_obstacle.position_cm
		object_yaw_deg = config_obstacle.yaw_deg
	calibration_camera, calibration_resolution = resolve_effective_camera_calibration(args, source)

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
	yolo_worker = LatestFrameYoloWorker(model, estimator, args.imgsz, args.conf, inference_device, args.yolo_fps)
	slam_pump = None if args.slam_sync else LatestFrameSlamPump(slam_backend, args.slam_fps, input_mode=args.slam_input)
	slam_period = (1.0 / args.slam_fps) if args.slam_fps > 0.0 else 0.0
	last_slam_push_at = None
	slam_fps_ema = 0.0
	last_result = None
	last_detections = []
	last_detection = None
	last_detection_obstacle = None
	last_yolo_frame = None
	last_yolo_sequence = 0
	last_yolo_fps = 0.0
	last_inference_ms = 0.0
	last_camera_world_pose = None
	last_camera_local_detection_cm = None
	slam_pose = None
	pose_fusion = PoseFusion()
	fused_pose = None
	path_projection = None

	try:
		slam_backend.start()
		yolo_worker.start()
		if slam_pump is not None:
			slam_pump.start()
		while True:
			frame_start = perf_counter()
			read_start = perf_counter()
			ok, frame = capture.read()
			read_ms = (perf_counter() - read_start) * 1000.0
			if not ok or frame is None:
				break
			yolo_worker.submit(frame)
			if slam_pump is not None:
				slam_pump.submit(frame)
				current_slam_fps = slam_pump.fps()
				slam_transport_status = slam_pump.stats_text()
			else:
				slam_now = perf_counter()
				slam_due = slam_period <= 0.0 or last_slam_push_at is None or (slam_now - last_slam_push_at) >= slam_period
				if slam_due:
					slam_fps_ema, last_slam_push_at = _update_fps_ema(slam_fps_ema, last_slam_push_at, slam_now)
					slam_frame = preprocess_slam_frame(frame, args.slam_input)
					slam_backend.push_frame(slam_frame)
					slam_transport_status = f"slam sync {slam_frame_stats(slam_frame)}"
				else:
					slam_transport_status = f"slam sync wait {slam_frame_stats(preprocess_slam_frame(frame, args.slam_input))}"
				current_slam_fps = slam_fps_ema

			postprocess_start = perf_counter()
			new_slam_pose = slam_backend.poll()
			if new_slam_pose is not None:
				slam_pose = new_slam_pose
				path_projection = nearest_path_point(flight_path_points, slam_pose.position_cm)
			now = time.time()
			command_result = command_tracker.current_pose(now) if command_tracker is not None else None
			command_pose = command_result.camera_world_pose if command_result is not None else None

			camera_world_pose = None
			yolo_output = yolo_worker.poll()
			new_yolo_measurement = yolo_output is not None and int(yolo_output["sequence"]) != last_yolo_sequence
			if new_yolo_measurement:
				last_yolo_sequence = int(yolo_output["sequence"])
				last_yolo_frame = yolo_output["frame"]
				last_result = yolo_output["result"]
				reference_detection = last_detection
				last_detections = (
					estimator.estimate_all_from_result(last_result, reference_detection=reference_detection)
					if last_result is not None
					else []
				)
				last_inference_ms = float(yolo_output["inference_ms"])
				last_yolo_fps = float(yolo_output["fps"])
				last_detection, last_detection_obstacle, camera_world_pose = select_pose_detection(
					detections=last_detections,
					scene_map=scene_map,
					object_id=args.object_id,
					config_obstacle=config_obstacle,
					object_position_cm=object_position_cm,
					object_yaw_deg=object_yaw_deg,
					command_pose=command_pose,
					last_camera_world_pose=last_camera_world_pose,
					slam_pose=slam_pose,
				)

			result = last_result
			detections = last_detections
			detection = last_detection
			detection_obstacle = last_detection_obstacle
			reference_obstacle = detection_obstacle or config_obstacle
			dashboard_object_position_cm = object_position_cm
			dashboard_object_yaw_deg = object_yaw_deg
			if args.object_position_cm is None and reference_obstacle is not None:
				dashboard_object_position_cm = reference_obstacle.position_cm
				dashboard_object_yaw_deg = reference_obstacle.yaw_deg
			if camera_world_pose is not None:
				last_camera_world_pose = camera_world_pose
				if slam_pose is None:
					path_projection = nearest_path_point(flight_path_points, camera_world_pose.position_cm)
			if detection is not None and detection.position_cm is not None:
				last_camera_local_detection_cm = detection.position_cm
			fused_result = pose_fusion.update(
				slam_pose=slam_pose,
				pnp_pose=camera_world_pose,
				pnp_confidence=detection.pose_confidence if new_yolo_measurement and detection is not None else 0.0,
				command_pose=command_pose,
				command_noise_cm=args.command_noise_cm,
				flight_path_points=flight_path_points,
				timestamp=now,
			)
			if fused_result is not None:
				fused_pose = fused_result.camera_world_pose
				path_projection = nearest_path_point(flight_path_points, fused_pose.position_cm)
			postprocess_ms = (perf_counter() - postprocess_start) * 1000.0
			dashboard = build_horizontal_dashboard(
				frame=frame,
				result=result,
				detection=detection,
				detections=detections,
				estimator=estimator,
				model_name=str(args.model),
				panel_height=args.panel_height,
				read_ms=read_ms,
				inference_ms=last_inference_ms,
				postprocess_ms=postprocess_ms,
				fps=fps_ema,
				object_position_cm=dashboard_object_position_cm,
				object_yaw_deg=dashboard_object_yaw_deg,
				slam_status=f"{slam_backend.status_text()} {slam_transport_status}",
				yolo_fps=last_yolo_fps,
				slam_fps=current_slam_fps,
				vision_frame=last_yolo_frame,
			)

			frame_end = perf_counter()
			frame_seconds = frame_end - frame_start
			if frame_seconds > 0:
				instant_fps = 1.0 / frame_seconds
				fps_ema = instant_fps if fps_ema == 0.0 else (fps_ema * 0.9 + instant_fps * 0.1)
			else:
				fps_ema = 0.0

			cv2.imshow(args.window_name, dashboard)
			if scene_3d is not None:
				display_camera_world_pose = camera_world_pose if camera_world_pose is not None else last_camera_world_pose
				if args.slam_only_viz:
					visible_camera_pose = slam_pose.camera_world_pose if slam_pose is not None else None
					pose_markers = build_pose_markers(
						None,
						slam_pose,
						None,
						None,
						path_projection,
						pose_confidence=0.0,
						slam_status=f"{slam_backend.status_text()} {slam_transport_status}",
					)
				else:
					visible_camera_pose = (
						fused_pose
						if fused_pose is not None
						else display_camera_world_pose
						if display_camera_world_pose is not None
						else slam_pose.camera_world_pose
						if slam_pose is not None
						else None
					)
					pose_markers = build_pose_markers(
						display_camera_world_pose,
						slam_pose,
						fused_pose,
						command_pose,
						path_projection,
						pose_confidence=detection.pose_confidence if detection is not None else 0.0,
						slam_status=f"{slam_backend.status_text()} {slam_transport_status}",
					)
				scene_3d.show(
					scene_map,
					active_obstacle=None if args.slam_only_viz else detection_obstacle,
					camera_pose=visible_camera_pose,
					camera_local_detection_cm=(
						None
						if args.slam_only_viz
						else detection.position_cm
						if detection is not None and detection.position_cm is not None
						else last_camera_local_detection_cm
					),
					flight_path_points=flight_path_points,
					pose_markers=pose_markers,
				)
			key = cv2.waitKey(1) & 0xFF
			if scene_3d is not None and hasattr(scene_3d, "handle_key"):
				scene_3d.handle_key(key)
			if key in (27, ord("q")):
				break
	finally:
		yolo_stopped = yolo_worker.stop()
		slam_stopped = True if slam_pump is None else slam_pump.stop()
		if not yolo_stopped:
			print("warning: YOLO worker did not stop within timeout")
		if not slam_stopped:
			print("warning: SLAM worker did not stop within timeout; skipping slam backend close")
		capture.release()
		if slam_stopped:
			slam_backend.close()
		cv2.destroyAllWindows()


if __name__ == "__main__":
	main()
