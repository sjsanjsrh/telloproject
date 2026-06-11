from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from yolo_seg.flight_plan import nearest_path_point
from yolo_seg.slam_backend import SlamPose
from yolo_seg.world_pose import CameraWorldPose, yaw_rotation_matrix_z


GOOD_SLAM_STATES = {"OK", "TRACKING"}

COMMAND_SPEED_MIN_CM_S = 1.0
COMMAND_TRACKER_DEFAULT_SPEED_CM_S = 50.0
COMMAND_DEFAULT_NOISE_CM = 45.0

FUSION_PROCESS_NOISE = 25.0
FUSION_INITIAL_POSITION_NOISE = 2500.0
FUSION_INITIAL_VELOCITY_NOISE = 10000.0
FUSION_DT_MIN_SEC = 1e-3
FUSION_DT_MAX_SEC = 0.5

PNP_MIN_CONFIDENCE = 0.22
PNP_NOISE_HIGH_CM = 280.0
PNP_NOISE_LOW_CM = 60.0

SLAM_DELTA_MAX_CM = 150.0
SLAM_DT_MIN_SEC = 1e-3
SLAM_DT_MAX_SEC = 0.5
SLAM_VELOCITY_NOISE_CM_S = 180.0

PATH_PRIOR_NOISE_CM = 450.0


@dataclass
class FusedPose:
	camera_world_pose: CameraWorldPose
	status: str


@dataclass
class CommandPose:
	camera_world_pose: CameraWorldPose
	status: str
	segment_index: int


class CommandPoseTracker:
	"""Interpolate the expected command pose along flight path points."""

	def __init__(self, flight_path_points: list[dict], speed_cm_s: float = COMMAND_TRACKER_DEFAULT_SPEED_CM_S, start_index: int = 0):
		self.points = list(flight_path_points or [])
		self.speed_cm_s = max(COMMAND_SPEED_MIN_CM_S, float(speed_cm_s))
		self.start_index = max(0, int(start_index))
		self.start_time: Optional[float] = None
		self._segments = self._build_segments()

	def _build_segments(self):
		if len(self.points) < 1:
			return []
		positions = [np.asarray(point["position_cm"], dtype=np.float64).reshape(3) for point in self.points]
		segments = []
		for index in range(len(positions) - 1):
			start = positions[index]
			end = positions[index + 1]
			delta = end - start
			length = float(np.linalg.norm(delta))
			start_yaw_deg = float(self.points[index].get("yaw_deg", 0.0))
			end_yaw_deg = float(self.points[index + 1].get("yaw_deg", start_yaw_deg))
			segments.append((index, start, end, delta, length, start_yaw_deg, end_yaw_deg))
		return segments

	def current_pose(self, timestamp: float) -> Optional[CommandPose]:
		if not self.points:
			return None
		if self.start_time is None:
			self.start_time = float(timestamp)

		if not self._segments:
			position = np.asarray(self.points[0]["position_cm"], dtype=np.float64).reshape(3)
			yaw_deg = float(self.points[0].get("yaw_deg", 0.0))
			return CommandPose(
				camera_world_pose=CameraWorldPose(tuple(float(value) for value in position), yaw_rotation_matrix_z(yaw_deg), "command:start"),
				status="command start",
				segment_index=0,
			)

		remaining = max(0.0, float(timestamp) - self.start_time) * self.speed_cm_s
		for index, start, end, delta, length, start_yaw_deg, end_yaw_deg in self._segments[self.start_index :]:
			if length <= 1e-6:
				continue
			if remaining <= length:
				t = remaining / length
				position = start + delta * t
				yaw_delta = ((end_yaw_deg - start_yaw_deg + 180.0) % 360.0) - 180.0
				yaw_deg = start_yaw_deg + yaw_delta * t
				return CommandPose(
					camera_world_pose=CameraWorldPose(
						position_cm=tuple(float(value) for value in position),
						rotation_matrix=yaw_rotation_matrix_z(yaw_deg),
						method="command:path",
					),
					status=f"command {index + 1}",
					segment_index=index,
				)
			remaining -= length

		position = np.asarray(self.points[-1]["position_cm"], dtype=np.float64).reshape(3)
		yaw_deg = float(self.points[-1].get("yaw_deg", 0.0))
		return CommandPose(
			camera_world_pose=CameraWorldPose(tuple(float(value) for value in position), yaw_rotation_matrix_z(yaw_deg), "command:end"),
			status="command end",
			segment_index=len(self._segments) - 1,
		)


class PoseFusion:
	"""Small position Kalman filter for PnP anchors, SLAM deltas, and path prior."""

	def __init__(
		self,
		process_noise: float = FUSION_PROCESS_NOISE,
		initial_position_noise: float = FUSION_INITIAL_POSITION_NOISE,
		initial_velocity_noise: float = FUSION_INITIAL_VELOCITY_NOISE,
		pnp_min_confidence: float = PNP_MIN_CONFIDENCE,
		pnp_noise_high_cm: float = PNP_NOISE_HIGH_CM,
		pnp_noise_low_cm: float = PNP_NOISE_LOW_CM,
	):
		self.x = np.zeros((6, 1), dtype=np.float64)
		self.P = np.diag(
			[
				initial_position_noise,
				initial_position_noise,
				initial_position_noise,
				initial_velocity_noise,
				initial_velocity_noise,
				initial_velocity_noise,
			]
		).astype(np.float64)
		self.process_noise = float(process_noise)
		self.pnp_min_confidence = float(pnp_min_confidence)
		self.pnp_noise_high_cm = float(pnp_noise_high_cm)
		self.pnp_noise_low_cm = float(pnp_noise_low_cm)
		self.initialized = False
		self.last_time: Optional[float] = None
		self.last_slam_pose: Optional[SlamPose] = None
		self.last_slam_time: Optional[float] = None
		self.last_rotation = np.eye(3, dtype=np.float64)
		self.last_status = "fusion init"

	def predict(self, timestamp: float) -> float:
		if self.last_time is None:
			self.last_time = float(timestamp)
			return 0.0

		dt = max(FUSION_DT_MIN_SEC, min(FUSION_DT_MAX_SEC, float(timestamp) - self.last_time))
		self.last_time = float(timestamp)
		F = np.eye(6, dtype=np.float64)
		F[0, 3] = dt
		F[1, 4] = dt
		F[2, 5] = dt

		q = self.process_noise
		Q = np.diag([q * dt * dt, q * dt * dt, q * dt * dt, q, q, q]).astype(np.float64)
		self.x = F @ self.x
		self.P = F @ self.P @ F.T + Q
		return dt

	def _update(self, z, H, R):
		z = np.asarray(z, dtype=np.float64).reshape(-1, 1)
		H = np.asarray(H, dtype=np.float64)
		R = np.asarray(R, dtype=np.float64)
		y = z - H @ self.x
		S = H @ self.P @ H.T + R
		K = self.P @ H.T @ np.linalg.inv(S)
		self.x = self.x + K @ y
		I = np.eye(self.P.shape[0], dtype=np.float64)
		self.P = (I - K @ H) @ self.P

	def update_position(self, position_cm, noise_cm: float):
		H = np.zeros((3, 6), dtype=np.float64)
		H[0, 0] = 1.0
		H[1, 1] = 1.0
		H[2, 2] = 1.0
		R = np.eye(3, dtype=np.float64) * (float(noise_cm) ** 2)
		self._update(position_cm, H, R)

	def update_velocity(self, velocity_cm_s, noise_cm_s: float):
		H = np.zeros((3, 6), dtype=np.float64)
		H[0, 3] = 1.0
		H[1, 4] = 1.0
		H[2, 5] = 1.0
		R = np.eye(3, dtype=np.float64) * (float(noise_cm_s) ** 2)
		self._update(velocity_cm_s, H, R)

	def reset_slam_anchor(self):
		self.last_slam_pose = None
		self.last_slam_time = None

	def update(
		self,
		slam_pose: Optional[SlamPose],
		pnp_pose: Optional[CameraWorldPose],
		pnp_confidence: float = 0.0,
		command_pose: Optional[CameraWorldPose] = None,
		command_noise_cm: float = COMMAND_DEFAULT_NOISE_CM,
		flight_path_points: Optional[list[dict]] = None,
		timestamp: Optional[float] = None,
	) -> Optional[FusedPose]:
		now = float(timestamp if timestamp is not None else (slam_pose.timestamp if slam_pose is not None else 0.0))
		if now <= 0.0:
			import time

			now = time.time()
		dt = self.predict(now)
		parts: list[str] = []
		rotation_source = None
		pnp_rotation_candidate = None

		if command_pose is not None:
			command_position = np.asarray(command_pose.position_cm, dtype=np.float64).reshape(3)
			self.last_rotation = np.asarray(command_pose.rotation_matrix, dtype=np.float64).reshape(3, 3)
			rotation_source = "cmd rot"
			if not self.initialized:
				self.x[:3, 0] = command_position
				self.x[3:, 0] = 0.0
				self.initialized = True
				parts.append("cmd init")
			else:
				self.update_position(command_position, noise_cm=command_noise_cm)
				parts.append("cmd")

		if pnp_pose is not None:
			pnp_position = np.asarray(pnp_pose.position_cm, dtype=np.float64).reshape(3)
			if not self.initialized and pnp_confidence >= self.pnp_min_confidence:
				pnp_rotation_candidate = np.asarray(pnp_pose.rotation_matrix, dtype=np.float64).reshape(3, 3)
				self.x[:3, 0] = pnp_position
				self.x[3:, 0] = 0.0
				self.initialized = True
				parts.append("pnp init")
			elif pnp_confidence >= self.pnp_min_confidence:
				pnp_rotation_candidate = np.asarray(pnp_pose.rotation_matrix, dtype=np.float64).reshape(3, 3)
				noise = float(
					np.interp(
						np.clip(pnp_confidence, self.pnp_min_confidence, 1.0),
						[self.pnp_min_confidence, 1.0],
						[self.pnp_noise_high_cm, self.pnp_noise_low_cm],
					)
				)
				self.update_position(pnp_position, noise_cm=noise)
				parts.append("pnp")
			else:
				parts.append("pnp reject")

		if slam_pose is not None:
			state = slam_pose.tracking_state.upper()
			if state in GOOD_SLAM_STATES and self.initialized:
				if rotation_source is None:
					self.last_rotation = np.asarray(slam_pose.rotation_matrix, dtype=np.float64).reshape(3, 3)
					rotation_source = "slam rot"
				if self.last_slam_pose is not None and self.last_slam_time is not None and dt > 1e-3:
					current = np.asarray(slam_pose.position_cm, dtype=np.float64).reshape(3)
					previous = np.asarray(self.last_slam_pose.position_cm, dtype=np.float64).reshape(3)
					slam_dt = max(SLAM_DT_MIN_SEC, min(SLAM_DT_MAX_SEC, float(slam_pose.timestamp) - self.last_slam_time))
					delta = current - previous
					if float(np.linalg.norm(delta)) < SLAM_DELTA_MAX_CM:
						self.update_velocity(delta / slam_dt, noise_cm_s=SLAM_VELOCITY_NOISE_CM_S)
						parts.append("slam d")
				self.last_slam_pose = slam_pose
				self.last_slam_time = float(slam_pose.timestamp)
			else:
				parts.append("slam hold")
		else:
			parts.append("slam hold")

		if rotation_source is None and pnp_rotation_candidate is not None:
			self.last_rotation = pnp_rotation_candidate
			rotation_source = "pnp rot"
		if rotation_source is not None:
			parts.append(rotation_source)

		if self.initialized and flight_path_points:
			path_projection = nearest_path_point(flight_path_points, self.x[:3, 0])
			if path_projection is not None:
				self.update_position(path_projection["position_cm"], noise_cm=PATH_PRIOR_NOISE_CM)
				parts.append("path")

		if not self.initialized:
			return None

		self.last_status = "+".join(parts) if parts else "predict"
		return FusedPose(
			camera_world_pose=CameraWorldPose(
				position_cm=tuple(float(value) for value in self.x[:3, 0]),
				rotation_matrix=self.last_rotation,
				method=f"fusion:{self.last_status}",
			),
			status=self.last_status,
		)
