from __future__ import annotations

from pathlib import Path
from typing import Any, Optional

import numpy as np
import yaml

from yolo_seg.world_pose import yaw_rotation_matrix_z


def _float3(value: Any, field_name: str) -> tuple[float, float, float]:
	if not isinstance(value, (list, tuple)) or len(value) != 3:
		raise ValueError(f"{field_name} must be a 3-value list")
	return float(value[0]), float(value[1]), float(value[2])


def load_flight_plan(path: str | Path) -> Optional[dict[str, Any]]:
	plan_path = Path(path)
	if not plan_path.exists():
		return None
	with open(plan_path, "r", encoding="utf-8") as file_handle:
		return yaml.safe_load(file_handle) or {}


def flight_plan_points(plan: Optional[dict[str, Any]]) -> list[dict[str, Any]]:
	if not plan:
		return []

	start = plan.get("start", {}) or {}
	position = np.asarray(_float3(start.get("position_cm", [0.0, 0.0, 0.0]), "start.position_cm"), dtype=np.float64)
	yaw_deg = float(start.get("yaw_deg", 0.0))
	points = [
		{
			"name": str(start.get("position_name", "start")),
			"position_cm": tuple(float(value) for value in position),
			"kind": "start",
		}
	]

	if start.get("takeoff_position_cm") is not None:
		takeoff_offset = np.asarray(_float3(start["takeoff_position_cm"], "start.takeoff_position_cm"), dtype=np.float64)
		position = position + takeoff_offset
		points.append(
			{
				"name": str(start.get("takeoff_name", "takeoff")),
				"position_cm": tuple(float(value) for value in position),
				"kind": "takeoff",
			}
		)
	elif start.get("takeoff_height_cm") is not None:
		position = position + np.array([0.0, 0.0, float(start["takeoff_height_cm"])], dtype=np.float64)
		points.append(
			{
				"name": str(start.get("takeoff_name", "takeoff")),
				"position_cm": tuple(float(value) for value in position),
				"kind": "takeoff",
			}
		)

	if start.get("move_cm") is not None:
		move = np.asarray(_float3(start["move_cm"], "start.move_cm"), dtype=np.float64)
		position = position + yaw_rotation_matrix_z(yaw_deg) @ move
		points.append(
			{
				"name": str(start.get("name", "hover")),
				"position_cm": tuple(float(value) for value in position),
				"kind": "waypoint",
			}
		)

	if start.get("rotate_deg") is not None:
		yaw_deg += float(start["rotate_deg"])

	for index, step in enumerate(plan.get("waypoints", []), start=1):
		name = str(step.get("name", f"waypoint {index}"))
		if step.get("position_cm") is not None:
			position = np.asarray(_float3(step["position_cm"], f"{name}.position_cm"), dtype=np.float64)
			points.append({"name": name, "position_cm": tuple(float(value) for value in position), "kind": "waypoint"})
		elif step.get("move_cm") is not None:
			move = np.asarray(_float3(step["move_cm"], f"{name}.move_cm"), dtype=np.float64)
			position = position + yaw_rotation_matrix_z(yaw_deg) @ move
			points.append({"name": name, "position_cm": tuple(float(value) for value in position), "kind": "waypoint"})

		if step.get("rotate_deg") is not None:
			yaw_deg += float(step["rotate_deg"])

	return points
