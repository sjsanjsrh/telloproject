from __future__ import annotations

import json
import sys
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

import cv2
import numpy as np

from yolo_seg.world_pose import CameraWorldPose, yaw_rotation_matrix_z


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


def _latest_json_record(path: Path) -> Optional[dict[str, Any]]:
	if not path.exists():
		return None
	text = path.read_text(encoding="utf-8").strip()
	if not text:
		return None
	if text.startswith("{"):
		return json.loads(text)
	for line in reversed(text.splitlines()):
		line = line.strip()
		if line:
			return json.loads(line)
	return None


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


class FileSlamBackend(SlamBackend):
	"""Poll a JSON/JSONL pose file written by a real SLAM process."""

	def __init__(self, pose_file: str | Path, scale_to_cm: float = 100.0, source: str = "slam"):
		self.pose_file = Path(pose_file)
		self.scale_to_cm = float(scale_to_cm)
		self.source = source
		self._last_timestamp: Optional[float] = None
		self._last_error: Optional[str] = None

	def poll(self) -> Optional[SlamPose]:
		record = _latest_json_record(self.pose_file)
		if record is None:
			return None
		try:
			pose = parse_slam_pose(record, scale_to_cm=self.scale_to_cm, source=self.source)
		except Exception as exc:
			self._last_error = str(exc)
			return None
		self._last_error = None
		if self._last_timestamp is not None and pose.timestamp <= self._last_timestamp:
			return None
		self._last_timestamp = pose.timestamp
		return pose

	def status_text(self) -> str:
		if self._last_error:
			return f"pose parse error: {self._last_error}"
		if not self.pose_file.exists():
			return "waiting pose file"
		size = self.pose_file.stat().st_size
		if size <= 0:
			return "waiting pose"
		return "pose file ready"


class OrbSlam3ProcessBackend(FileSlamBackend):
	"""Start an external ORB-SLAM3 live executable and read its exported pose file.

	The executable must be a real ORB-SLAM3 wrapper that writes JSON or JSONL
	records to --pose-out. This Python side intentionally does not fake tracking.
	"""

	def __init__(
		self,
		executable: str | Path,
		vocabulary: str | Path,
		settings: str | Path,
		pose_file: str | Path,
		relay_frame_file: str | Path | None = None,
		mode: str = "mono",
		source: str = "tello",
		scale_to_cm: float = 100.0,
	):
		super().__init__(pose_file=pose_file, scale_to_cm=scale_to_cm, source="orbslam3")
		self.executable = Path(executable)
		self.vocabulary = Path(vocabulary)
		self.settings = Path(settings)
		self.relay_frame_file = Path(relay_frame_file) if relay_frame_file is not None else None
		self.mode = mode
		self.video_source = source
		self.process: subprocess.Popen | None = None
		self._relay_frames_written = 0
		self._relay_last_shape: tuple[int, int] | None = None
		self._relay_last_write_time: float | None = None

	def start(self) -> None:
		if not self.executable.exists():
			raise FileNotFoundError(f"ORB-SLAM3 executable not found: {self.executable}")
		if not self.vocabulary.exists():
			raise FileNotFoundError(f"ORB-SLAM3 vocabulary not found: {self.vocabulary}")
		if not self.settings.exists():
			raise FileNotFoundError(f"ORB-SLAM3 settings not found: {self.settings}")

		self.pose_file.parent.mkdir(parents=True, exist_ok=True)
		self.pose_file.write_text("", encoding="utf-8")
		command = [
			str(self.executable),
			"--mode",
			self.mode,
			"--vocab",
			str(self.vocabulary),
			"--settings",
			str(self.settings),
			"--source",
			self.video_source,
			"--pose-out",
			str(self.pose_file),
		]
		self.process = subprocess.Popen(command)

	def push_frame(self, frame) -> None:
		if self.relay_frame_file is None or frame is None:
			return
		self.relay_frame_file.parent.mkdir(parents=True, exist_ok=True)
		height, width = frame.shape[:2]
		encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 92])[1]
		temp_path = self.relay_frame_file.with_suffix(".jpg.tmp")
		temp_path.write_bytes(encoded.tobytes())
		temp_path.replace(self.relay_frame_file)
		self._relay_frames_written += 1
		self._relay_last_shape = (int(width), int(height))
		self._relay_last_write_time = time.time()

	def status_text(self) -> str:
		process_text = "not started"
		if self.process is not None:
			code = self.process.poll()
			process_text = "running" if code is None else f"exited {code}"
		pose_text = super().status_text()
		if self.relay_frame_file is None:
			return f"{process_text}, {pose_text}"
		if not self.relay_frame_file.exists():
			return f"{process_text}, waiting relay frame"
		size = self.relay_frame_file.stat().st_size
		frame_text = f"relay {size}B"
		if self._relay_last_shape is not None:
			width, height = self._relay_last_shape
			frame_text = f"{frame_text}, frames {self._relay_frames_written}, {width}x{height}"
		if self._relay_last_write_time is not None:
			age_ms = (time.time() - self._relay_last_write_time) * 1000.0
			frame_text = f"{frame_text}, age {age_ms:.0f}ms"
		return f"{process_text}, {frame_text}, {pose_text}"

	def close(self) -> None:
		if self.process is None:
			return
		if self.process.poll() is None:
			self.process.terminate()
			try:
				self.process.wait(timeout=2.0)
			except subprocess.TimeoutExpired:
				self.process.kill()
			self.process = None


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
			self._last_pose = parse_slam_pose(record, scale_to_cm=self.scale_to_cm, source="orbslam3_py")
			self._frames_tracked += 1
			self._last_shape = (int(width), int(height))
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
		Path("third_party") / "orbslam3_live" / "ORBvoc.bin",
		Path("third_party") / "orbslam3_live" / "ORBvoc.txt",
	]
	for candidate in candidates:
		if candidate.exists():
			return candidate
	return candidates[0]


def create_slam_backend(args) -> SlamBackend:
	backend = getattr(args, "slam_backend", "none")
	if backend == "none":
		return SlamBackend()
	if backend == "file":
		if not args.slam_pose_file:
			raise ValueError("--slam-pose-file is required for --slam-backend file")
		return FileSlamBackend(args.slam_pose_file, scale_to_cm=args.slam_scale_cm)
	if backend in {"orbslam3", "orbslam3_py"}:
		executable = args.orbslam3_exe or (Path("third_party") / "orbslam3_live" / "orbslam3_live.exe")
		vocabulary = args.orbslam3_vocab or _default_orbslam3_vocabulary()
		settings = args.orbslam3_settings
		if settings is None:
			resolution = getattr(args, "video_resolution", 480)
			settings = Path("yolo_seg") / f"orbslam3_tello_forward_{resolution}p.yaml"
		if backend == "orbslam3_py":
			return OrbSlam3PythonBackend(
				module_path=getattr(args, "orbslam3_py_module", None),
				vocabulary=vocabulary,
				settings=settings,
				scale_to_cm=args.slam_scale_cm,
			)
		relay_frame_file = getattr(args, "orbslam3_relay_frame_file", None)
		if relay_frame_file is None:
			relay_frame_file = Path("yolo_seg") / "runtime" / "orbslam3_frame.jpg"
		source = args.orbslam3_source
		if source == "auto":
			video_source = str(getattr(args, "source", "0"))
			source = f"relay:{relay_frame_file}" if video_source.strip().lower() == "tello" else video_source
		pose_file = args.slam_pose_file or (Path("yolo_seg") / "runtime" / "orbslam3_pose.jsonl")
		return OrbSlam3ProcessBackend(
			executable=executable,
			vocabulary=vocabulary,
			settings=settings,
			pose_file=pose_file,
			relay_frame_file=relay_frame_file if str(source).startswith("relay:") else None,
			mode=args.orbslam3_mode,
			source=source,
			scale_to_cm=args.slam_scale_cm,
		)
	raise ValueError(f"Unknown SLAM backend: {backend}")
