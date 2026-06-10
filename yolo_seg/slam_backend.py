from __future__ import annotations

import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

import numpy as np

from camera_calibration.camera_tranceform import load_camera_params, make_profile_name
from yolo_seg.world_pose import CameraWorldPose, opencv_to_tello_rotation, opencv_to_tello_vector, yaw_rotation_matrix_z

GOOD_SLAM_STATES = {"OK", "TRACKING"}


@dataclass
class SlamPose:
	position_cm: tuple[float, float, float]
	rotation_matrix: np.ndarray
	tracking_state: str = "UNKNOWN"
	timestamp: float = 0.0
	source: str = "slam"

	@property
	def camera_world_pose(self) -> CameraWorldPose:
		return CameraWorldPose(
			position_cm=self.position_cm,
			rotation_matrix=self.rotation_matrix,
			method=f"{self.source}:{self.tracking_state}",
		)


class SlamBackend:
	def start(self) -> None:
		pass

	def poll(self) -> Optional[SlamPose]:
		return None

	def push_frame(self, frame) -> None:
		pass

	def status_text(self) -> str:
		return "off"

	def close(self) -> None:
		pass


def rotation_from_yaw_pitch_roll(yaw_deg: float = 0.0, pitch_deg: float = 0.0, roll_deg: float = 0.0) -> np.ndarray:
	"""Build a Tello/world rotation matrix: X forward, Y right, Z up."""

	yaw = yaw_rotation_matrix_z(yaw_deg)
	pitch = np.deg2rad(float(pitch_deg))
	roll = np.deg2rad(float(roll_deg))
	pitch_matrix = np.array(
		[
			[np.cos(pitch), 0.0, np.sin(pitch)],
			[0.0, 1.0, 0.0],
			[-np.sin(pitch), 0.0, np.cos(pitch)],
		],
		dtype=np.float64,
	)
	roll_matrix = np.array(
		[
			[1.0, 0.0, 0.0],
			[0.0, np.cos(roll), -np.sin(roll)],
			[0.0, np.sin(roll), np.cos(roll)],
		],
		dtype=np.float64,
	)
	return yaw @ pitch_matrix @ roll_matrix


def parse_slam_pose(record: dict[str, Any], scale_to_cm: float = 1.0, source: str = "slam") -> SlamPose:
	position = record.get("position_cm")
	scale = 1.0
	if position is None:
		position = record.get("position")
		scale = float(scale_to_cm)
	if position is None:
		state = str(record.get("tracking_state", record.get("state", "UNKNOWN"))).upper()
		if state in {"LOST", "WAIT", "WAITING", "NOT_INITIALIZED"}:
			position = (0.0, 0.0, 0.0)
			scale = 1.0
		else:
			raise ValueError("SLAM pose record needs position_cm or position")
	position_array = np.asarray(position, dtype=np.float64).reshape(3) * scale

	rotation = record.get("rotation_matrix")
	if rotation is None:
		rotation_matrix = rotation_from_yaw_pitch_roll(
			yaw_deg=float(record.get("yaw_deg", 0.0)),
			pitch_deg=float(record.get("pitch_deg", 0.0)),
			roll_deg=float(record.get("roll_deg", 0.0)),
		)
	else:
		rotation_matrix = np.asarray(rotation, dtype=np.float64).reshape(3, 3)

	return SlamPose(
		position_cm=tuple(float(value) for value in position_array),
		rotation_matrix=rotation_matrix,
		tracking_state=str(record.get("tracking_state", record.get("state", "OK"))),
		timestamp=float(record.get("timestamp", time.time())),
		source=source,
	)


def convert_orbslam3_record_to_tello(record: dict[str, Any]) -> dict[str, Any]:
	"""Convert ORB-SLAM3/OpenCV camera axes to Tello axes for front camera use.

	ORB/OpenCV camera axes are X right, Y down, Z forward.
	Project Tello axes are X forward, Y right, Z up.
	"""

	converted = dict(record)
	if record.get("position") is not None:
		converted["position"] = opencv_to_tello_vector(record["position"]).tolist()
	if record.get("position_cm") is not None:
		converted["position_cm"] = opencv_to_tello_vector(record["position_cm"]).tolist()
	if record.get("rotation_matrix") is not None:
		converted["rotation_matrix"] = opencv_to_tello_rotation(record["rotation_matrix"]).tolist()
	return converted


class OrbSlam3PythonBackend(SlamBackend):
	"""Run ORB-SLAM3 in-process through orbslam3_py.pyd."""

	def __init__(
		self,
		module_path: str | Path | None,
		vocabulary: str | Path,
		settings: str | Path,
		scale_to_cm: float = 100.0,
	):
		self.module_path = Path(module_path) if module_path else self._find_default_module()
		self.vocabulary = Path(vocabulary)
		self.settings = Path(settings)
		self.scale_to_cm = float(scale_to_cm)
		self._module = None
		self._tracker = None
		self._last_pose: Optional[SlamPose] = None
		self._last_record: Optional[dict[str, Any]] = None
		self._frames_tracked = 0
		self._last_shape: tuple[int, int] | None = None
		self._last_error: Optional[str] = None

	@staticmethod
	def _find_default_module() -> Path:
		module_dir = Path("third_party") / "orbslam3_py"
		candidates = sorted(module_dir.glob("orbslam3_py*.pyd"))
		return candidates[0] if candidates else module_dir / "orbslam3_py.pyd"

	def start(self) -> None:
		if not self.module_path.exists():
			raise FileNotFoundError(f"ORB-SLAM3 Python module not found: {self.module_path}")
		if not self.vocabulary.exists():
			raise FileNotFoundError(f"ORB-SLAM3 vocabulary not found: {self.vocabulary}")
		if not self.settings.exists():
			raise FileNotFoundError(f"ORB-SLAM3 settings not found: {self.settings}")

		module_dir = str(self.module_path.parent.resolve())
		if module_dir not in sys.path:
			sys.path.insert(0, module_dir)
		import orbslam3_py  # type: ignore

		self._module = orbslam3_py
		self._tracker = orbslam3_py.OrbSlam3Mono(str(self.vocabulary), str(self.settings), False)

	def push_frame(self, frame) -> None:
		if self._tracker is None or frame is None:
			return
		try:
			height, width = frame.shape[:2]
			record = self._tracker.track(frame, time.time())
			record = convert_orbslam3_record_to_tello(record)
			self._last_record = record
			self._frames_tracked += 1
			self._last_shape = (int(width), int(height))
			state = str(record.get("tracking_state", record.get("state", "UNKNOWN"))).upper()
			self._last_pose = (
				parse_slam_pose(record, scale_to_cm=self.scale_to_cm, source="orbslam3_py")
				if state in GOOD_SLAM_STATES
				else None
			)
			self._last_error = None
		except Exception as exc:
			self._last_error = str(exc)

	def poll(self) -> Optional[SlamPose]:
		return self._last_pose

	def status_text(self) -> str:
		if self._last_error:
			return f"inproc error: {self._last_error}"
		if self._tracker is None:
			return "inproc not started"
		frame_text = f"inproc frames {self._frames_tracked}"
		if self._last_shape is not None:
			width, height = self._last_shape
			frame_text = f"{frame_text}, {width}x{height}"
		if self._last_pose is not None:
			frame_text = f"{frame_text}, {self._last_pose.tracking_state}"
		elif self._last_record is not None:
			state = str(self._last_record.get("tracking_state", self._last_record.get("state", "UNKNOWN")))
			frame_text = f"{frame_text}, {state}"
		return frame_text

	def close(self) -> None:
		if self._tracker is None:
			return
		try:
			self._tracker.shutdown()
		finally:
			self._tracker = None


def _default_orbslam3_vocabulary() -> Path:
	candidates = [
		Path("third_party") / "orbslam3_windows" / "ORB_SLAM3" / "Vocabulary" / "ORBvoc.bin",
		Path("third_party") / "orbslam3_windows" / "ORB_SLAM3" / "Vocabulary" / "ORBvoc.txt",
	]
	for candidate in candidates:
		if candidate.exists():
			return candidate
	return candidates[0]


def _replace_or_append_yaml_value(lines: list[str], key: str, value) -> list[str]:
	replacement = f"{key}: {value}\n"
	for index, line in enumerate(lines):
		if line.strip().startswith(f"{key}:"):
			lines[index] = replacement
			return lines
	lines.append(replacement)
	return lines


def _format_yaml_float(value: float) -> str:
	return f"{float(value):.12g}"


def _resolve_runtime_camera_selection(args) -> tuple[Optional[str], Optional[str], Optional[int]]:
	camera_profile = getattr(args, "effective_camera_profile", None) or getattr(args, "camera_profile", None)
	camera_direction = (
		getattr(args, "effective_camera_direction", None)
		or getattr(args, "camera", None)
		or getattr(args, "camera_direction", None)
	)
	resolution = (
		getattr(args, "effective_resolution", None)
		or getattr(args, "resolution", None)
		or getattr(args, "video_resolution", None)
	)
	return camera_profile, camera_direction, resolution


def _make_runtime_orbslam3_settings(base_settings: str | Path, args) -> Path:
	base_settings = Path(base_settings)
	with open(base_settings, "r", encoding="utf-8") as file_handle:
		lines = file_handle.readlines()

	camera_profile, camera_direction, resolution = _resolve_runtime_camera_selection(args)

	camera_params = getattr(args, "camera_params", None)
	if camera_params:
		camera_matrix, dist_coeffs = load_camera_params(
			camera_params,
			camera_profile=camera_profile,
			camera_direction=camera_direction,
			resolution=resolution,
		)
		dist = np.asarray(dist_coeffs, dtype=np.float64).reshape(-1)
		dist_values = [float(dist[index]) if index < len(dist) else 0.0 for index in range(5)]
		camera_values = {
			"Camera.fx": camera_matrix[0, 0],
			"Camera.fy": camera_matrix[1, 1],
			"Camera.cx": camera_matrix[0, 2],
			"Camera.cy": camera_matrix[1, 2],
			"Camera.k1": dist_values[0],
			"Camera.k2": dist_values[1],
			"Camera.p1": dist_values[2],
			"Camera.p2": dist_values[3],
			"Camera.k3": dist_values[4],
		}
		for key, value in camera_values.items():
			lines = _replace_or_append_yaml_value(lines, key, _format_yaml_float(value))

	slam_fps = float(getattr(args, "slam_fps", 0.0) or 0.0)
	if slam_fps > 0.0:
		lines = _replace_or_append_yaml_value(lines, "Camera.fps", _format_yaml_float(slam_fps))

	runtime_dir = Path("yolo_seg") / ".runtime"
	runtime_dir.mkdir(parents=True, exist_ok=True)
	profile = make_profile_name(
		camera_direction=camera_direction,
		resolution=resolution,
		camera_profile=camera_profile,
	) or "default"
	fps_tag = f"{slam_fps:g}" if slam_fps > 0.0 else "template"
	runtime_path = runtime_dir / f"{base_settings.stem}_{profile}_fps{fps_tag}.yaml"
	with open(runtime_path, "w", encoding="utf-8") as file_handle:
		file_handle.writelines(lines)
	print(
		f"ORB-SLAM3 runtime settings: {runtime_path} "
		f"camera={camera_direction or 'default'} resolution={resolution or 'default'} "
		f"fps={fps_tag}"
	)
	return runtime_path


def create_slam_backend(args) -> SlamBackend:
	backend = getattr(args, "slam_backend", "none")
	if backend == "none":
		return SlamBackend()
	if backend == "orbslam3_py":
		vocabulary = args.orbslam3_vocab or _default_orbslam3_vocabulary()
		settings = args.orbslam3_settings
		if settings is None:
			_, _, resolution = _resolve_runtime_camera_selection(args)
			resolution = resolution or 480
			settings = Path("yolo_seg") / f"orbslam3_tello_forward_{resolution}p.yaml"
		settings = _make_runtime_orbslam3_settings(settings, args)
		return OrbSlam3PythonBackend(
			module_path=getattr(args, "orbslam3_py_module", None),
			vocabulary=vocabulary,
			settings=settings,
			scale_to_cm=args.slam_scale_cm,
		)
	raise ValueError(f"Unknown SLAM backend: {backend}")
