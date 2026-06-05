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
	pose_method: str = "pnp"
	thickness_px: Optional[float] = None

	@property
	def position_cm(self) -> Optional[tuple[float, float, float]]:
		if self.tvec is None:
			return None
		flat = self.tvec.reshape(-1)
		return float(flat[0]), float(flat[1]), float(flat[2])


@dataclass
class CameraWorldPose:
	"""Camera pose estimated from a known target pose in world coordinates."""

	position_cm: tuple[float, float, float]
	rotation_matrix: np.ndarray
	method: str


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


def yaw_rotation_matrix_y(yaw_deg: float) -> np.ndarray:
	"""Rotation around OpenCV's vertical image/object Y axis."""

	yaw = np.deg2rad(float(yaw_deg))
	cos_yaw = np.cos(yaw)
	sin_yaw = np.sin(yaw)
	return np.array(
		[
			[cos_yaw, 0.0, sin_yaw],
			[0.0, 1.0, 0.0],
			[-sin_yaw, 0.0, cos_yaw],
		],
		dtype=np.float64,
	)


def estimate_camera_world_pose(
	detection: SquarePoseDetection,
	object_position_cm: Sequence[float],
	object_yaw_deg: float = 0.0,
) -> Optional[CameraWorldPose]:
	"""Estimate camera world pose from a known target world pose.

	OpenCV solvePnP returns object-to-camera pose: X_camera = R * X_object + t.
	This function inverts that transform, then places it at the known object pose.
	For rough outline detections without rvec, it assumes object and camera axes
	are roughly aligned and uses tvec only.
	"""

	if detection.tvec is None:
		return None

	object_position = np.asarray(object_position_cm, dtype=np.float64).reshape(3)
	t_obj_in_cam = np.asarray(detection.tvec, dtype=np.float64).reshape(3, 1)
	if detection.rvec is None:
		r_obj_to_cam = np.eye(3, dtype=np.float64)
	else:
		r_obj_to_cam, _jacobian = cv2.Rodrigues(np.asarray(detection.rvec, dtype=np.float64))

	r_cam_to_obj = r_obj_to_cam.T
	camera_position_obj = (-r_cam_to_obj @ t_obj_in_cam).reshape(3)

	r_world_obj = yaw_rotation_matrix_y(object_yaw_deg)
	camera_position_world = object_position + (r_world_obj @ camera_position_obj)
	r_world_cam = r_world_obj @ r_cam_to_obj
	return CameraWorldPose(
		position_cm=tuple(float(value) for value in camera_position_world),
		rotation_matrix=r_world_cam,
		method=detection.pose_method,
	)


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


def contours_from_mask(mask: np.ndarray, output_shape: Optional[tuple[int, int]] = None) -> list[np.ndarray]:
	"""Extract outer contours from a binary mask."""
	binary_mask = (np.asarray(mask) > 0).astype(np.uint8)
	if binary_mask.ndim != 2:
		raise ValueError("contours_from_mask expects a 2D mask")
	if output_shape is not None and binary_mask.shape[:2] != output_shape:
		binary_mask = cv2.resize(
			binary_mask,
			(output_shape[1], output_shape[0]),
			interpolation=cv2.INTER_NEAREST,
		)

	contours, _hierarchy = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	contours = [contour for contour in contours if cv2.contourArea(contour) > 0]
	contours.sort(key=cv2.contourArea, reverse=True)
	return contours


class YoloSquarePoseEstimator:
	"""Estimate the pose of a square from a YOLO segmentation output."""

	def __init__(
		self,
		camera_matrix: np.ndarray,
		dist_coeffs: np.ndarray,
		square_size_cm: float,
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
			return False
		return self._matches_class(class_id, class_name, names, self.outline_class_id, self.outline_class_name)

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
			if is_outline:
				rvec, tvec, thickness_px = self._estimate_pose_from_outline(contour, center_px)
				pose_method = "outline_thickness"
			else:
				rvec, tvec = self._estimate_pose(corners_px)
				thickness_px = None
				pose_method = "pnp"
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
				pose_method=pose_method,
				thickness_px=thickness_px,
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
		target_class_id=target_class_id,
		target_class_name=target_class_name,
		outline_class_id=outline_class_id,
		outline_class_name=outline_class_name,
		outline_thickness_cm=outline_thickness_cm,
		min_area_px=min_area_px,
	)
	return estimator.estimate_from_result(result)
