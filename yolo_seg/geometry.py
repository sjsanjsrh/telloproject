from __future__ import annotations

from typing import Optional, Sequence

import cv2
import numpy as np


RANSAC_LINE_ITERATIONS = 120
RANSAC_LINE_DISTANCE_PX = 3.0
RANSAC_MIN_INLIERS = 12
RANSAC_MIN_REMAINING_POINTS = 20


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

	return np.asarray(polygon, dtype=np.float32).reshape(-1, 1, 2)


def contour_center(contour: np.ndarray) -> tuple[float, float]:
	moments = cv2.moments(contour)
	if moments["m00"] != 0:
		return float(moments["m10"] / moments["m00"]), float(moments["m01"] / moments["m00"])
	pts = contour.reshape(-1, 2)
	center = pts.mean(axis=0)
	return float(center[0]), float(center[1])


def line_from_points(point_a: np.ndarray, point_b: np.ndarray) -> Optional[np.ndarray]:
	"""Return normalized ax + by + c = 0 line from two points."""

	x1, y1 = point_a
	x2, y2 = point_b
	a = y1 - y2
	b = x2 - x1
	c = x1 * y2 - x2 * y1
	norm = float(np.hypot(a, b))
	if norm <= 1e-6:
		return None
	return np.array([a / norm, b / norm, c / norm], dtype=np.float64)


def fit_line_tls(points: np.ndarray) -> Optional[np.ndarray]:
	"""Fit ax + by + c = 0 with total least squares."""

	pts = np.asarray(points, dtype=np.float64).reshape(-1, 2)
	if pts.shape[0] < 2:
		return None
	centroid = pts.mean(axis=0)
	centered = pts - centroid
	_, _singular_values, vh = np.linalg.svd(centered, full_matrices=False)
	direction = vh[0]
	normal = np.array([-direction[1], direction[0]], dtype=np.float64)
	norm = float(np.linalg.norm(normal))
	if norm <= 1e-6:
		return None
	normal /= norm
	c = -float(normal @ centroid)
	return np.array([normal[0], normal[1], c], dtype=np.float64)


def line_distances(points: np.ndarray, line: np.ndarray) -> np.ndarray:
	pts = np.asarray(points, dtype=np.float64).reshape(-1, 2)
	return np.abs(pts @ line[:2] + line[2])


def line_angle(line: np.ndarray) -> float:
	"""Line direction angle in [0, pi)."""

	angle = np.arctan2(-line[0], line[1])
	return float(angle % np.pi)


def angle_difference_mod_pi(angle_a: float, angle_b: float) -> float:
	diff = abs((angle_a - angle_b + np.pi / 2.0) % np.pi - np.pi / 2.0)
	return float(diff)


def line_intersection(line_a: np.ndarray, line_b: np.ndarray) -> Optional[np.ndarray]:
	a1, b1, c1 = line_a
	a2, b2, c2 = line_b
	det = a1 * b2 - a2 * b1
	if abs(det) <= 1e-6:
		return None
	x = (b1 * c2 - b2 * c1) / det
	y = (c1 * a2 - c2 * a1) / det
	return np.array([x, y], dtype=np.float32)


def ransac_fit_line(points: np.ndarray, rng: np.random.Generator) -> Optional[tuple[np.ndarray, np.ndarray]]:
	pts = np.asarray(points, dtype=np.float64).reshape(-1, 2)
	if pts.shape[0] < RANSAC_MIN_INLIERS:
		return None

	best_inliers = None
	best_count = 0
	for _ in range(RANSAC_LINE_ITERATIONS):
		indices = rng.choice(pts.shape[0], size=2, replace=False)
		line = line_from_points(pts[indices[0]], pts[indices[1]])
		if line is None:
			continue
		inliers = line_distances(pts, line) <= RANSAC_LINE_DISTANCE_PX
		count = int(np.count_nonzero(inliers))
		if count > best_count:
			best_count = count
			best_inliers = inliers

	if best_inliers is None or best_count < RANSAC_MIN_INLIERS:
		return None

	refined = fit_line_tls(pts[best_inliers])
	if refined is None:
		return None
	return refined, best_inliers


def ransac_contour_lines(points: np.ndarray) -> list[np.ndarray]:
	pts = np.asarray(points, dtype=np.float64).reshape(-1, 2)
	rng = np.random.default_rng(0)
	lines: list[np.ndarray] = []
	remaining = pts

	for _ in range(4):
		if remaining.shape[0] < RANSAC_MIN_REMAINING_POINTS:
			break
		result = ransac_fit_line(remaining, rng)
		if result is None:
			break
		line, inliers = result
		lines.append(line)
		remaining = remaining[~inliers]

	return lines


def corners_from_lines(lines: list[np.ndarray], contour_points: np.ndarray) -> Optional[np.ndarray]:
	if len(lines) < 4:
		return None

	angles = [line_angle(line) for line in lines]
	best_split = None
	best_score = float("inf")
	for i in range(4):
		for j in range(i + 1, 4):
			group_a = [i, j]
			group_b = [index for index in range(4) if index not in group_a]
			parallel_score = angle_difference_mod_pi(angles[group_a[0]], angles[group_a[1]])
			parallel_score += angle_difference_mod_pi(angles[group_b[0]], angles[group_b[1]])
			perp_score = abs(angle_difference_mod_pi(angles[group_a[0]], angles[group_b[0]]) - np.pi / 2.0)
			score = parallel_score + perp_score
			if score < best_score:
				best_score = score
				best_split = (group_a, group_b)

	if best_split is None:
		return None

	group_a, group_b = best_split
	intersections = []
	for idx_a in group_a:
		for idx_b in group_b:
			point = line_intersection(lines[idx_a], lines[idx_b])
			if point is not None and np.all(np.isfinite(point)):
				intersections.append(point)

	if len(intersections) != 4:
		return None

	corners = order_points(np.asarray(intersections, dtype=np.float32))
	contour_area = abs(float(cv2.contourArea(np.asarray(contour_points, dtype=np.float32).reshape(-1, 1, 2))))
	corner_area = abs(float(cv2.contourArea(corners.reshape(-1, 1, 2))))
	if corner_area <= 1.0 or contour_area <= 1.0:
		return None
	if corner_area < contour_area * 0.35:
		return None
	return corners


def contour_to_square_corners(contour: np.ndarray) -> Optional[np.ndarray]:
	"""Estimate four corners from all available contour vertices."""

	if contour is None or len(contour) < 4:
		return None

	contour_float = np.asarray(contour, dtype=np.float32)
	points = contour_float.reshape(-1, 2)
	lines = ransac_contour_lines(points)
	corners = corners_from_lines(lines, points)
	if corners is not None:
		return corners

	return None


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

	contours, _hierarchy = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
	contours = [contour for contour in contours if cv2.contourArea(contour) > 0]
	contours.sort(key=cv2.contourArea, reverse=True)
	return contours
