from __future__ import annotations

import json
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

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

	def poll(self) -> Optional[SlamPose]:
		record = _latest_json_record(self.pose_file)
		if record is None:
			return None
		pose = parse_slam_pose(record, scale_to_cm=self.scale_to_cm, source=self.source)
		if self._last_timestamp is not None and pose.timestamp <= self._last_timestamp:
			return None
		self._last_timestamp = pose.timestamp
		return pose


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
		mode: str = "mono",
		source: str = "tello",
		scale_to_cm: float = 100.0,
	):
		super().__init__(pose_file=pose_file, scale_to_cm=scale_to_cm, source="orbslam3")
		self.executable = Path(executable)
		self.vocabulary = Path(vocabulary)
		self.settings = Path(settings)
		self.mode = mode
		self.video_source = source
		self.process: subprocess.Popen | None = None

	def start(self) -> None:
		if not self.executable.exists():
			raise FileNotFoundError(f"ORB-SLAM3 executable not found: {self.executable}")
		if not self.vocabulary.exists():
			raise FileNotFoundError(f"ORB-SLAM3 vocabulary not found: {self.vocabulary}")
		if not self.settings.exists():
			raise FileNotFoundError(f"ORB-SLAM3 settings not found: {self.settings}")

		self.pose_file.parent.mkdir(parents=True, exist_ok=True)
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


def create_slam_backend(args) -> SlamBackend:
	backend = getattr(args, "slam_backend", "none")
	if backend == "none":
		return SlamBackend()
	if backend == "file":
		if not args.slam_pose_file:
			raise ValueError("--slam-pose-file is required for --slam-backend file")
		return FileSlamBackend(args.slam_pose_file, scale_to_cm=args.slam_scale_cm)
	if backend == "orbslam3":
		if not args.orbslam3_exe or not args.orbslam3_vocab or not args.orbslam3_settings:
			raise ValueError("--orbslam3-exe, --orbslam3-vocab, and --orbslam3-settings are required")
		pose_file = args.slam_pose_file or (Path("yolo_seg") / "runtime" / "orbslam3_pose.jsonl")
		return OrbSlam3ProcessBackend(
			executable=args.orbslam3_exe,
			vocabulary=args.orbslam3_vocab,
			settings=args.orbslam3_settings,
			pose_file=pose_file,
			mode=args.orbslam3_mode,
			source=args.orbslam3_source,
			scale_to_cm=args.slam_scale_cm,
		)
	raise ValueError(f"Unknown SLAM backend: {backend}")
