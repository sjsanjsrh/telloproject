from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np

from yolo_seg.world_pose import yaw_rotation_matrix_z


@dataclass
class WaypointCorrection:
	original_move_cm: tuple[int, int, int]
	corrected_move_cm: tuple[int, int, int]
	correction_local_cm: tuple[float, float, float]
	error_world_cm: tuple[float, float, float]
	applied: bool
	reason: str


def _as_float3(value: Sequence[float]) -> np.ndarray:
	return np.asarray(value, dtype=np.float64).reshape(3)


def _clamp_norm(vector: np.ndarray, max_norm: float) -> np.ndarray:
	norm = float(np.linalg.norm(vector))
	if norm <= max_norm or norm <= 1e-9:
		return vector
	return vector * (max_norm / norm)


class WaypointCorrector:
	"""Convert current pose error into a bounded correction for the next move."""

	def __init__(
		self,
		gain: float = 0.4,
		max_correction_cm: float = 25.0,
		min_confidence: float = 0.45,
		dry_run: bool = True,
	):
		self.gain = float(gain)
		self.max_correction_cm = float(max_correction_cm)
		self.min_confidence = float(min_confidence)
		self.dry_run = bool(dry_run)

	def correct_move(
		self,
		planned_move_cm: Sequence[float],
		commanded_position_cm: Sequence[float],
		estimated_position_cm: Optional[Sequence[float]],
		yaw_deg: float = 0.0,
		confidence: float = 1.0,
	) -> WaypointCorrection:
		planned = _as_float3(planned_move_cm)
		original_move = tuple(int(round(value)) for value in planned)
		if estimated_position_cm is None:
			return WaypointCorrection(original_move, original_move, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), False, "no pose")

		if float(confidence) < self.min_confidence:
			return WaypointCorrection(original_move, original_move, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), False, "low confidence")

		commanded = _as_float3(commanded_position_cm)
		estimated = _as_float3(estimated_position_cm)
		error_world = estimated - commanded
		correction_world = _clamp_norm(-self.gain * error_world, self.max_correction_cm)
		correction_local = yaw_rotation_matrix_z(float(yaw_deg)).T @ correction_world
		corrected = planned + correction_local
		corrected_move = tuple(int(round(value)) for value in corrected)

		return WaypointCorrection(
			original_move_cm=original_move,
			corrected_move_cm=original_move if self.dry_run else corrected_move,
			correction_local_cm=tuple(float(value) for value in correction_local),
			error_world_cm=tuple(float(value) for value in error_world),
			applied=not self.dry_run,
			reason="dry-run" if self.dry_run else "applied",
		)
