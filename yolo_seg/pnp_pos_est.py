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
from typing import Optional, Sequence

import cv2
import numpy as np


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

	@property
	def position_cm(self) -> Optional[tuple[float, float, float]]:
		if self.tvec is None:
			return None
		flat = self.tvec.reshape(-1)
		return float(flat[0]), float(flat[1]), float(flat[2])


def order_points(points: np.ndarray) -> np.ndarray:
	"""Order four points as top-left, top-right, bottom-right, bottom-left."""

	pts = np.asarray(points, dtype=np.float32).reshape(-1, 2)
	if pts.shape[0] != 4:
		raise ValueError("order_points expects exactly 4 points")

	ordered = np.zeros((4, 2), dtype=np.float32)
	point_sums = pts.sum(axis=1)
	point_diffs = np.diff(pts, axis=1).reshape(-1)

	ordered[0] = pts[np.argmin(point_sums)]
	ordered[2] = pts[np.argmax(point_sums)]
	ordered[1] = pts[np.argmin(point_diffs)]
	ordered[3] = pts[np.argmax(point_diffs)]
	return ordered


def contour_from_polygon(polygon: Sequence[Sequence[float]]) -> np.ndarray:
	"""Convert a polygon to an OpenCV contour."""

	contour = np.asarray(polygon, dtype=np.float32).reshape(-1, 1, 2)
	return contour


def contour_center(contour: np.ndarray) -> tuple[float, float]:
	moments = cv2.moments(contour)
	if moments["m00"] != 0:
		return float(moments["m10"] / moments["m00"]), float(moments["m01"] / moments["m00"])
	pts = contour.reshape(-1, 2)
	center = pts.mean(axis=0)
	return float(center[0]), float(center[1])


def contour_to_square_corners(contour: np.ndarray) -> Optional[np.ndarray]:
	"""Estimate four corners from a contour.

	We first try polygon approximation. If the contour does not simplify to a
	clean quadrilateral, we fall back to the minimum-area rectangle.
	"""

	if contour is None or len(contour) < 4:
		return None

	contour_float = np.asarray(contour, dtype=np.float32)
	perimeter = cv2.arcLength(contour_float, True)
	approx = cv2.approxPolyDP(contour_float, 0.02 * perimeter, True)

	if len(approx) == 4 and cv2.isContourConvex(approx):
		return order_points(approx.reshape(-1, 2))

	rect = cv2.minAreaRect(contour_float)
	box = cv2.boxPoints(rect)
	return order_points(box)


class YoloSquarePoseEstimator:
	"""Estimate the pose of a square from a YOLO segmentation output."""

	def __init__(
		self,
		camera_matrix: np.ndarray,
		dist_coeffs: np.ndarray,
		square_size_cm: float,
		target_class_id: Optional[int] = None,
		target_class_name: Optional[str] = None,
		min_area_px: float = 200.0,
	) -> None:
		self.camera_matrix = np.asarray(camera_matrix, dtype=np.float64)
		self.dist_coeffs = np.asarray(dist_coeffs, dtype=np.float64).reshape(-1, 1)
		self.square_size_cm = float(square_size_cm)
		self.target_class_id = target_class_id
		self.target_class_name = target_class_name
		self.min_area_px = float(min_area_px)

	def _class_matches(
		self,
		class_id: Optional[int],
		class_name: Optional[str],
		names: Optional[dict[int, str]],
	) -> bool:
		if self.target_class_id is not None and class_id is not None and class_id != self.target_class_id:
			return False
		if self.target_class_name is not None:
			if class_name is not None and class_name != self.target_class_name:
				return False
			if class_name is None and names is not None and class_id is not None:
				mapped_name = names.get(int(class_id))
				if mapped_name != self.target_class_name:
					return False
		return True

	def _extract_candidate_contours(self, result) -> list[tuple[np.ndarray, float, Optional[int], Optional[str]]]:
		"""Extract contours from an Ultralytics result or a mask-like object."""

		candidates: list[tuple[np.ndarray, float, Optional[int], Optional[str]]] = []
		names = getattr(result, "names", None)

		masks = getattr(result, "masks", None)
		boxes = getattr(result, "boxes", None)
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
		if mask_data is None:
			return candidates

		if hasattr(mask_data, "detach"):
			mask_array = mask_data.detach().cpu().numpy()
		else:
			mask_array = np.asarray(mask_data)

		if mask_array.ndim == 2:
			mask_array = mask_array[np.newaxis, ...]

		for index, mask in enumerate(mask_array):
			binary_mask = (mask > 0.5).astype(np.uint8) * 255
			contours, _ = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
			if not contours:
				continue
			contour = max(contours, key=cv2.contourArea)
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

	def _estimate_pose(self, corners_px: np.ndarray) -> tuple[Optional[np.ndarray], Optional[np.ndarray]]:
		half = self.square_size_cm / 2.0
		object_points = np.array(
			[
				[-half, -half, 0.0],
				[half, -half, 0.0],
				[half, half, 0.0],
				[-half, half, 0.0],
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

	def estimate_from_result(self, result) -> Optional[SquarePoseDetection]:
		"""Estimate the best square detection from a YOLO result object."""

		if isinstance(result, (list, tuple)):
			if not result:
				return None
			result = result[0]

		candidates = self._extract_candidate_contours(result)
		if not candidates:
			return None

		names = getattr(result, "names", None)
		best_detection: Optional[SquarePoseDetection] = None

		for contour, confidence, class_id, class_name in candidates:
			if not self._class_matches(class_id, class_name, names):
				continue

			contour_area = float(cv2.contourArea(contour))
			if contour_area < self.min_area_px:
				continue

			corners_px = contour_to_square_corners(contour)
			if corners_px is None:
				continue

			rvec, tvec = self._estimate_pose(corners_px)
			center_px = contour_center(contour)
			score = contour_area * max(confidence, 1e-6)

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
			)

			if best_detection is None or detection.score > best_detection.score:
				best_detection = detection

		return best_detection

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
	target_class_id: Optional[int] = None,
	target_class_name: Optional[str] = None,
	min_area_px: float = 200.0,
) -> Optional[SquarePoseDetection]:
	"""Convenience wrapper for one-shot square pose estimation."""

	estimator = YoloSquarePoseEstimator(
		camera_matrix=camera_matrix,
		dist_coeffs=dist_coeffs,
		square_size_cm=square_size_cm,
		target_class_id=target_class_id,
		target_class_name=target_class_name,
		min_area_px=min_area_px,
	)
	return estimator.estimate_from_result(result)
