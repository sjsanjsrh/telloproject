from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

import yaml


@dataclass(frozen=True)
class ObstacleShape:
	type: str
	width_cm: float
	height_cm: float
	thickness_cm: float
	diameter_cm: Optional[float] = None
	pillar_diameter_cm: Optional[float] = None
	pillar_height_cm: Optional[float] = None


@dataclass(frozen=True)
class Obstacle:
	id: str
	position_cm: tuple[float, float, float]
	yaw_deg: float
	shape: ObstacleShape
	class_id: Optional[int] = None
	class_name: Optional[str] = None
	color_bgr: Optional[tuple[int, int, int]] = None


@dataclass(frozen=True)
class SceneMap:
	unit: str
	axis: str
	obstacles: dict[str, Obstacle]

	def get(self, obstacle_id: str) -> Obstacle:
		try:
			return self.obstacles[obstacle_id]
		except KeyError as exc:
			available = ", ".join(sorted(self.obstacles))
			raise KeyError(f"Obstacle not found: {obstacle_id}. Available: {available}") from exc

	def match_detection(self, class_id: Optional[int], class_name: Optional[str]) -> Optional[Obstacle]:
		for obstacle in self.obstacles.values():
			if obstacle.class_id is not None and class_id == obstacle.class_id:
				return obstacle
			if obstacle.class_name is not None and class_name == obstacle.class_name:
				return obstacle
		return None


def _float3(value: Any, field_name: str) -> tuple[float, float, float]:
	if not isinstance(value, (list, tuple)) or len(value) != 3:
		raise ValueError(f"{field_name} must be a 3-value list")
	return float(value[0]), float(value[1]), float(value[2])


def _int3(value: Any, field_name: str) -> tuple[int, int, int]:
	if not isinstance(value, (list, tuple)) or len(value) != 3:
		raise ValueError(f"{field_name} must be a 3-value list")
	return int(value[0]), int(value[1]), int(value[2])


def _size_value(size: dict[str, Any], key: str) -> float:
	try:
		return float(size[key])
	except KeyError as exc:
		raise ValueError(f"shape.size_cm.{key} is required") from exc


def _first_float(*values: Any, default: float = 0.0) -> float:
	for value in values:
		if value is not None:
			return float(value)
	return float(default)


def load_scene_map(path: str | Path) -> SceneMap:
	with open(path, "r", encoding="utf-8") as file_handle:
		data = yaml.safe_load(file_handle) or {}

	world = data.get("world", {})
	unit = str(world.get("unit", "cm"))
	axis = str(world.get("axis", "opencv"))

	obstacles: dict[str, Obstacle] = {}
	for raw_obstacle in data.get("obstacles", []):
		obstacle_id = str(raw_obstacle["id"])
		raw_shape = raw_obstacle.get("shape", {})
		raw_size = raw_shape.get("size_cm", {})
		shape_type = str(raw_shape.get("type", raw_obstacle.get("type", "rectangle_frame")))
		width_cm = _first_float(raw_shape.get("width_cm"), raw_size.get("width"), raw_size.get("diameter"))
		height_cm = _first_float(raw_shape.get("height_cm"), raw_size.get("height"), raw_size.get("diameter"))
		diameter_cm = raw_shape.get("diameter_cm", raw_size.get("diameter"))
		pillar_diameter_cm = raw_shape.get("pillar_diameter_cm", raw_size.get("pillar_diameter"))
		pillar_height_cm = raw_shape.get("pillar_height_cm", raw_size.get("pillar_height"))
		if shape_type in {"landing_point", "landing_zone", "landing_marker", "circle_pillar"}:
			if diameter_cm is None:
				raise ValueError(f"{obstacle_id}.shape.size_cm.diameter is required for {shape_type}")
			width_cm = float(diameter_cm)
			height_cm = float(diameter_cm)
		if shape_type in {"start_area", "start_rectangle"}:
			if width_cm <= 0.0 or height_cm <= 0.0:
				raise ValueError(f"{obstacle_id}.shape.size_cm.width and height are required for {shape_type}")
		shape = ObstacleShape(
			type=shape_type,
			width_cm=width_cm if width_cm > 0.0 else _size_value(raw_size, "width"),
			height_cm=height_cm if height_cm > 0.0 else _size_value(raw_size, "height"),
			thickness_cm=float(raw_shape.get("thickness_cm", raw_obstacle.get("thickness_cm", 0.0))),
			diameter_cm=float(diameter_cm) if diameter_cm is not None else None,
			pillar_diameter_cm=float(pillar_diameter_cm) if pillar_diameter_cm is not None else None,
			pillar_height_cm=float(pillar_height_cm) if pillar_height_cm is not None else None,
		)
		obstacles[obstacle_id] = Obstacle(
			id=obstacle_id,
			position_cm=_float3(raw_obstacle["position_cm"], f"{obstacle_id}.position_cm"),
			yaw_deg=float(raw_obstacle.get("yaw_deg", 0.0)),
			shape=shape,
			class_id=int(raw_obstacle["class_id"]) if raw_obstacle.get("class_id") is not None else None,
			class_name=raw_obstacle.get("class_name"),
			color_bgr=_int3(raw_obstacle["color_bgr"], f"{obstacle_id}.color_bgr") if raw_obstacle.get("color_bgr") is not None else None,
		)

	return SceneMap(unit=unit, axis=axis, obstacles=obstacles)
