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
from yolo_seg.pnp_pos_est import SquarePoseDetection, YoloSquarePoseEstimator, estimate_camera_world_pose


DEFAULT_MODEL = "yolo_seg/res/runs/segment/train/weights/best.pt"
DEFAULT_CAMERA_PARAMS = Path("camera_calibration") / "camera_params.yaml"
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
	parser.add_argument("--object-position-cm", default=None, help="Known target world position as x,y,z in cm")
	parser.add_argument("--object-yaw-deg", type=float, default=0.0, help="Known target yaw angle in world coordinates")
	parser.add_argument("--camera-direction", default="forward", choices=("forward", "downward"), help="Tello camera direction when using --source tello")
	parser.add_argument("--video-resolution", type=int, default=480, choices=(480, 720), help="Tello video resolution when using --source tello")
	parser.add_argument("--min-area-px", type=float, default=200.0, help="Minimum contour area in pixels")
	parser.add_argument("--imgsz", type=int, default=640, help="Inference image size")
	parser.add_argument("--conf", type=float, default=0.25, help="Confidence threshold")
	parser.add_argument("--device", default="auto", help="Inference device, e.g. auto, intel:NPU, cpu")
	parser.add_argument("--panel-height", type=int, default=360, help="Output panel height")
	parser.add_argument("--window-name", default="pnp_pos_est_horizontal", help="OpenCV window name")
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
		raise ValueError("--object-position-cm must look like x,y,z")
	return float(parts[0]), float(parts[1]), float(parts[2])


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
		try:
			self.tello.__del__()
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

	label = f"{detection.pose_method} score={detection.score:.1f} area={detection.area_px:.0f}px conf={detection.confidence:.2f}"
	if detection.class_name:
		label = f"{detection.class_name} {label}"
	cv2.putText(output, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2, cv2.LINE_AA)

	if detection.position_cm is not None:
		x, y, z = detection.position_cm
		pose_label = f"tvec=({x:.1f}, {y:.1f}, {z:.1f}) cm"
		cv2.putText(output, pose_label, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2, cv2.LINE_AA)

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
) -> np.ndarray:
	original_panel = resize_to_height(frame, panel_height)
	plot_panel = resize_to_fit_height(build_outline_overlay(frame, result), panel_height) if result is not None else original_panel.copy()
	pose_panel = resize_to_height(add_detection_info(frame, detection), panel_height)

	if detection is None:
		status_lines = [
			f"model: {model_name}",
			"status: no valid square detection",
			f"read: {read_ms:.1f} ms",
			f"inference: {inference_ms:.1f} ms",
			f"post: {postprocess_ms:.1f} ms",
			f"fps: {fps:.1f}",
			f"min area: {estimator.min_area_px:.0f}px",
			f"square size: {estimator.square_size_cm:.1f}cm",
		]
	else:
		position = detection.position_cm
		if position is None:
			position_text = "tvec: unavailable"
		else:
			position_text = f"tvec cm: {position[0]:.1f}, {position[1]:.1f}, {position[2]:.1f}"
		thickness_text = "thickness px: n/a"
		if detection.thickness_px is not None:
			thickness_text = f"thickness px: {detection.thickness_px:.1f}"
		drone_position_text = "drone world: n/a"
		if object_position_cm is not None:
			camera_world_pose = estimate_camera_world_pose(detection, object_position_cm, object_yaw_deg)
			if camera_world_pose is not None:
				wx, wy, wz = camera_world_pose.position_cm
				drone_position_text = f"drone world cm: {wx:.1f}, {wy:.1f}, {wz:.1f}"
		outer_rect = cv2.minAreaRect(detection.contour)
		outer_w, outer_h = outer_rect[1]
		status_lines = [
			f"model: {model_name}",
			f"target: {detection.class_name or detection.class_id or 'square'}",
			f"method: {detection.pose_method}",
			f"read: {read_ms:.1f} ms",
			f"inference: {inference_ms:.1f} ms",
			f"post: {postprocess_ms:.1f} ms",
			f"fps: {fps:.1f}",
			f"score: {detection.score:.1f}   conf: {detection.confidence:.2f}",
			f"range px: {outer_w:.1f} x {outer_h:.1f}",
			f"center px: ({detection.center_px[0]:.1f}, {detection.center_px[1]:.1f})",
			thickness_text,
			position_text,
			drone_position_text,
		]

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
	object_position_cm = parse_vector3(args.object_position_cm)
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
		target_class_id=args.target_class_id,
		target_class_name=args.target_class_name,
		outline_class_id=args.outline_class_id,
		outline_class_name=args.outline_class_name,
		outline_thickness_cm=args.outline_thickness_cm,
		min_area_px=args.min_area_px,
	)

	model, inference_device = _resolve_inference_model(args.model, args.device, args.imgsz)
	capture = open_capture(source, args.camera_direction, args.video_resolution)
	fps_ema = 0.0
	last_postprocess_ms = 0.0

	try:
		while True:
			frame_start = perf_counter()
			read_start = perf_counter()
			ok, frame = capture.read()
			read_ms = (perf_counter() - read_start) * 1000.0
			if not ok or frame is None:
				break

			inference_start = perf_counter()
			results = model.predict(frame, imgsz=args.imgsz, conf=args.conf, device=inference_device, verbose=False)
			inference_ms = (perf_counter() - inference_start) * 1000.0
			result = results[0] if results else None

			postprocess_start = perf_counter()
			detection = estimator.estimate_from_result(result) if result is not None else None
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
				object_position_cm=object_position_cm,
				object_yaw_deg=args.object_yaw_deg,
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
			key = cv2.waitKey(1) & 0xFF
			if key in (27, ord("q")):
				break
	finally:
		capture.release()
		cv2.destroyAllWindows()


if __name__ == "__main__":
	main()
