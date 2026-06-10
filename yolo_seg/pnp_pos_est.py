"""Square pose estimation from YOLO segmentation results.

This module converts a segmented square-like mask into a 4-corner polygon and
estimates the square pose with OpenCV's solvePnP.

Typical usage:

	estimator = YoloSquarePoseEstimator(camera_matrix, dist_coeffs, square_size_cm=20.0)
	detection = estimator.estimate_from_result(result)
	if detection is not None:
		print(detection.position_cm)
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np

from yolo_seg.geometry import (
	contour_center,
	contour_from_polygon,
	contour_to_square_corners,
	contours_from_mask,
)
from yolo_seg.world_pose import opencv_to_tello_vector


@dataclass
class SquarePoseDetection:
	"""Pose estimation result for one square target."""

	center_px: tuple[float, float]
	corners_px: np.ndarray
	contour: np.ndarray
	area_px: float
	confidence: float
	score: float
	rvec: Optional[np.ndarray]
	tvec: Optional[np.ndarray]
	class_id: Optional[int] = None
	class_name: Optional[str] = None
	pose_method: str = "pnp"
	thickness_px: Optional[float] = None
	pose_confidence: float = 0.0
	corner_count: int = 0
	avoidance_only: bool = False

	@property
	def position_cm(self) -> Optional[tuple[float, float, float]]:
		if self.tvec is None:
			return None
		flat = self.tvec.reshape(-1)
		return float(flat[0]), float(flat[1]), float(flat[2])

	@property
	def position_tello_cm(self) -> Optional[tuple[float, float, float]]:
		if self.tvec is None:
			return None
		flat = opencv_to_tello_vector(self.tvec)
		return float(flat[0]), float(flat[1]), float(flat[2])


class YoloSquarePoseEstimator:
	"""Estimate the pose of a square from a YOLO segmentation output."""

	def __init__(
		self,
		camera_matrix: np.ndarray,
		dist_coeffs: np.ndarray,
		square_size_cm: float,
		target_width_cm: Optional[float] = None,
		target_height_cm: Optional[float] = None,
		target_class_id: Optional[int] = None,
		target_class_name: Optional[str] = None,
		outline_class_id: Optional[int] = None,
		outline_class_name: Optional[str] = None,
		outline_thickness_cm: Optional[float] = None,
		min_area_px: float = 200.0,
	) -> None:
		self.camera_matrix = np.asarray(camera_matrix, dtype=np.float64)
		self.dist_coeffs = np.asarray(dist_coeffs, dtype=np.float64).reshape(-1, 1)
		self.square_size_cm = float(square_size_cm)
		self.target_width_cm = float(target_width_cm) if target_width_cm is not None else self.square_size_cm
		self.target_height_cm = float(target_height_cm) if target_height_cm is not None else self.square_size_cm
		self.target_class_id = target_class_id
		self.target_class_name = target_class_name
		self.outline_class_id = outline_class_id
		self.outline_class_name = outline_class_name
		self.outline_thickness_cm = float(outline_thickness_cm) if outline_thickness_cm is not None else None
		self.min_area_px = float(min_area_px)

	def _matches_class(
		self,
		class_id: Optional[int],
		class_name: Optional[str],
		names: Optional[dict[int, str]],
		target_class_id: Optional[int],
		target_class_name: Optional[str],
	) -> bool:
		if target_class_id is not None and class_id is not None and class_id != target_class_id:
			return False
		if target_class_name is not None:
			if class_name is not None and class_name != target_class_name:
				return False
			if class_name is None and names is not None and class_id is not None:
				mapped_name = names.get(int(class_id))
				if mapped_name != target_class_name:
					return False
		return True

	def _class_matches(
		self,
		class_id: Optional[int],
		class_name: Optional[str],
		names: Optional[dict[int, str]],
	) -> bool:
		return self._matches_class(class_id, class_name, names, self.target_class_id, self.target_class_name)

	def _outline_class_matches(
		self,
		class_id: Optional[int],
		class_name: Optional[str],
		names: Optional[dict[int, str]],
	) -> bool:
		if self.outline_thickness_cm is None:
			return False
		if self.outline_class_id is None and self.outline_class_name is None:
			return class_name in {"partial_gate", "partinal_gate"}
		return self._matches_class(class_id, class_name, names, self.outline_class_id, self.outline_class_name)

	def _visible_corner_count(self, contour: np.ndarray) -> int:
		contour_float = np.asarray(contour, dtype=np.float32)
		if len(contour_float) < 4:
			return int(len(contour_float))
		perimeter = cv2.arcLength(contour_float, True)
		if perimeter <= 1e-6:
			return 0
		approx = cv2.approxPolyDP(contour_float, 0.025 * perimeter, True)
		return int(len(approx))

	def _extract_candidate_contours(self, result) -> list[tuple[np.ndarray, float, Optional[int], Optional[str]]]:
		"""Extract contours from an Ultralytics result or a mask-like object."""

		candidates: list[tuple[np.ndarray, float, Optional[int], Optional[str]]] = []
		names = getattr(result, "names", None)

		masks = getattr(result, "masks", None)
		boxes = getattr(result, "boxes", None)
		orig_shape = getattr(result, "orig_shape", None)
		if orig_shape is not None:
			orig_shape = tuple(int(value) for value in orig_shape[:2])
		if masks is not None and getattr(masks, "xy", None) is not None:
			polygons = masks.xy
			for index, polygon in enumerate(polygons):
				contour = contour_from_polygon(polygon)
				confidence = 1.0
				class_id = None
				class_name = None
				if boxes is not None and len(boxes) > index:
					box_conf = boxes.conf[index]
					confidence = float(box_conf.item() if hasattr(box_conf, "item") else box_conf)
					class_tensor = boxes.cls[index]
					class_id = int(class_tensor.item() if hasattr(class_tensor, "item") else class_tensor)
					if isinstance(names, dict):
						class_name = names.get(class_id)
				candidates.append((contour, confidence, class_id, class_name))
			return candidates

		mask_data = getattr(masks, "data", None) if masks is not None else None
		if mask_data is not None:
			if hasattr(mask_data, "detach"):
				mask_array = mask_data.detach().cpu().numpy()
			else:
				mask_array = np.asarray(mask_data)

			if mask_array.ndim == 2:
				mask_array = mask_array[np.newaxis, ...]

			for index, mask in enumerate(mask_array):
				contours = contours_from_mask(mask, output_shape=orig_shape)
				if not contours:
					continue

				confidence = 1.0
				class_id = None
				class_name = None
				if boxes is not None and len(boxes) > index:
					box_conf = boxes.conf[index]
					confidence = float(box_conf.item() if hasattr(box_conf, "item") else box_conf)
					class_tensor = boxes.cls[index]
					class_id = int(class_tensor.item() if hasattr(class_tensor, "item") else class_tensor)
					if isinstance(names, dict):
						class_name = names.get(class_id)

				for contour in contours:
					candidates.append((contour, confidence, class_id, class_name))
			return candidates

		if mask_data is None:
			return candidates

		return candidates

	def _estimate_pose(self, corners_px: np.ndarray) -> tuple[Optional[np.ndarray], Optional[np.ndarray]]:
		half_width = self.target_width_cm / 2.0
		half_height = self.target_height_cm / 2.0
		object_points = np.array(
			[
				[-half_width, -half_height, 0.0],
				[half_width, -half_height, 0.0],
				[half_width, half_height, 0.0],
				[-half_width, half_height, 0.0],
			],
			dtype=np.float32,
		)

		image_points = np.asarray(corners_px, dtype=np.float32).reshape(4, 2)
		ok, rvec, tvec = cv2.solvePnP(
			object_points,
			image_points,
			self.camera_matrix,
			self.dist_coeffs,
			flags=cv2.SOLVEPNP_ITERATIVE,
		)
		if not ok:
			return None, None
		return rvec, tvec

	def _estimate_pose_from_outline(
		self,
		contour: np.ndarray,
		center_px: tuple[float, float],
	) -> tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[float]]:
		if self.outline_thickness_cm is None:
			return None, None, None

		rect = cv2.minAreaRect(np.asarray(contour, dtype=np.float32))
		width_px, height_px = rect[1]
		thickness_px = float(min(width_px, height_px))
		if thickness_px <= 0:
			return None, None, None

		f = float((self.camera_matrix[0, 0] + self.camera_matrix[1, 1]) / 2.0)
		fx = float(self.camera_matrix[0, 0])
		fy = float(self.camera_matrix[1, 1])
		cx = float(self.camera_matrix[0, 2])
		cy = float(self.camera_matrix[1, 2])

		z_cm = f * self.outline_thickness_cm / thickness_px
		u, v = center_px
		x_cm = (float(u) - cx) / fx * z_cm
		y_cm = (float(v) - cy) / fy * z_cm
		tvec = np.array([[x_cm], [y_cm], [z_cm]], dtype=np.float64)
		return None, tvec, thickness_px

	def _pose_confidence(
		self,
		pose_method: str,
		detection_confidence: float,
		area_px: float,
		has_pose: bool,
	) -> float:
		if not has_pose:
			return 0.0

		area_quality = min(1.0, max(0.0, area_px / max(self.min_area_px * 8.0, 1.0)))
		det_quality = min(1.0, max(0.0, detection_confidence))
		method_weight = 1.0 if pose_method == "pnp" else 0.28
		quality = (0.35 + 0.65 * det_quality) * (0.35 + 0.65 * area_quality)
		return float(min(1.0, method_weight * quality))

	def estimate_all_from_result(self, result) -> list[SquarePoseDetection]:
		"""Estimate every usable gate/partial-gate candidate from a YOLO result object."""

		if isinstance(result, (list, tuple)):
			if not result:
				return []
			result = result[0]

		candidates = self._extract_candidate_contours(result)
		if not candidates:
			return []

		names = getattr(result, "names", None)
		detections: list[SquarePoseDetection] = []

		for contour, confidence, class_id, class_name in candidates:
			is_outline = self._outline_class_matches(class_id, class_name, names)
			if not is_outline and not self._class_matches(class_id, class_name, names):
				continue

			contour_area = float(cv2.contourArea(contour))
			if contour_area < self.min_area_px:
				continue

			corners_px = contour_to_square_corners(contour)
			if corners_px is None:
				continue

			center_px = contour_center(contour)
			corner_count = self._visible_corner_count(contour)
			avoidance_only = bool(is_outline and corner_count <= 3)
			if avoidance_only:
				rvec, tvec, thickness_px = None, None, None
				pose_method = "partial_avoidance"
			elif is_outline:
				rvec, tvec, thickness_px = self._estimate_pose_from_outline(contour, center_px)
				pose_method = "outline_thickness"
			else:
				rvec, tvec = self._estimate_pose(corners_px)
				thickness_px = None
				pose_method = "pnp"
			score = contour_area * max(confidence, 1e-6)
			pose_confidence = self._pose_confidence(
				pose_method=pose_method,
				detection_confidence=float(confidence),
				area_px=contour_area,
				has_pose=tvec is not None,
			)

			detection = SquarePoseDetection(
				center_px=center_px,
				corners_px=corners_px,
				contour=np.asarray(contour, dtype=np.float32),
				area_px=contour_area,
				confidence=float(confidence),
				score=float(score),
				rvec=rvec,
				tvec=tvec,
				class_id=class_id,
				class_name=class_name,
				pose_method=pose_method,
				thickness_px=thickness_px,
				pose_confidence=pose_confidence,
				corner_count=corner_count,
				avoidance_only=avoidance_only,
			)

			detections.append(detection)

		detections.sort(key=lambda detection: detection.score, reverse=True)
		return detections

	def estimate_from_result(self, result) -> Optional[SquarePoseDetection]:
		"""Estimate the best non-avoidance square detection from a YOLO result object."""

		for detection in self.estimate_all_from_result(result):
			if not detection.avoidance_only and detection.tvec is not None:
				return detection
		return None

	def draw_detection(self, frame: np.ndarray, detection: SquarePoseDetection) -> np.ndarray:
		"""Draw the detection on a frame and return the annotated image."""

		output = frame.copy()
		corners = detection.corners_px.astype(np.int32).reshape(-1, 1, 2)
		cv2.polylines(output, [corners], True, (0, 255, 0), 2)
		center = tuple(int(round(value)) for value in detection.center_px)
		cv2.circle(output, center, 4, (0, 0, 255), -1)

		label = f"square conf={detection.confidence:.2f}"
		if detection.class_name:
			label = f"{detection.class_name} {label}"
		cv2.putText(output, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
		return output


def estimate_square_pose_from_result(
	result,
	camera_matrix: np.ndarray,
	dist_coeffs: np.ndarray,
	square_size_cm: float,
	target_width_cm: Optional[float] = None,
	target_height_cm: Optional[float] = None,
	target_class_id: Optional[int] = None,
	target_class_name: Optional[str] = None,
	outline_class_id: Optional[int] = None,
	outline_class_name: Optional[str] = None,
	outline_thickness_cm: Optional[float] = None,
	min_area_px: float = 200.0,
) -> Optional[SquarePoseDetection]:
	"""Convenience wrapper for one-shot square pose estimation."""

	estimator = YoloSquarePoseEstimator(
		camera_matrix=camera_matrix,
		dist_coeffs=dist_coeffs,
		square_size_cm=square_size_cm,
		target_width_cm=target_width_cm,
		target_height_cm=target_height_cm,
		target_class_id=target_class_id,
		target_class_name=target_class_name,
		outline_class_id=outline_class_id,
		outline_class_name=outline_class_name,
		outline_thickness_cm=outline_thickness_cm,
		min_area_px=min_area_px,
	)
	return estimator.estimate_from_result(result)
