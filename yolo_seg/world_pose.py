from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence

import cv2
import numpy as np


OPENCV_TO_TELLO = np.array(
	[
		[0.0, 0.0, 1.0],
		[1.0, 0.0, 0.0],
		[0.0, -1.0, 0.0],
	],
	dtype=np.float64,
)
TELLO_TO_OPENCV = OPENCV_TO_TELLO.T


@dataclass
class CameraWorldPose:
	"""Camera pose estimated from a known target pose in world coordinates."""

	position_cm: tuple[float, float, float]
	rotation_matrix: np.ndarray
	method: str


def opencv_to_tello_vector(vector) -> np.ndarray:
	"""Convert an OpenCV camera/object vector to Tello axes.

	Tello axes used by this project: X forward, Y right, Z up.
	OpenCV axes: X image-right, Y image-down, Z camera-forward.
	"""

	return OPENCV_TO_TELLO @ np.asarray(vector, dtype=np.float64).reshape(3)


def opencv_to_tello_rotation(rotation_matrix: np.ndarray) -> np.ndarray:
	"""Convert a rotation matrix from OpenCV axes to Tello axes."""

	return OPENCV_TO_TELLO @ np.asarray(rotation_matrix, dtype=np.float64).reshape(3, 3) @ TELLO_TO_OPENCV


def yaw_rotation_matrix_z(yaw_deg: float) -> np.ndarray:
	"""Rotation around Tello/world Z-up yaw axis."""

	yaw = np.deg2rad(float(yaw_deg))
	cos_yaw = np.cos(yaw)
	sin_yaw = np.sin(yaw)
	return np.array(
		[
			[cos_yaw, -sin_yaw, 0.0],
			[sin_yaw, cos_yaw, 0.0],
			[0.0, 0.0, 1.0],
		],
		dtype=np.float64,
	)


def estimate_camera_world_pose(
	detection,
	object_position_cm: Sequence[float],
	object_yaw_deg: float = 0.0,
) -> Optional[CameraWorldPose]:
	"""Estimate camera world pose from a known object pose.

	OpenCV solvePnP returns object-to-camera pose: X_camera = R * X_object + t.
	This function converts that pose to Tello axes, inverts it, then places it
	at the known object pose. Project Tello/world axes are X forward, Y right,
	and Z up.
	"""

	if detection.tvec is None:
		return None

	object_position = np.asarray(object_position_cm, dtype=np.float64).reshape(3)
	t_obj_in_cam = np.asarray(detection.tvec, dtype=np.float64).reshape(3, 1)
	if detection.rvec is None:
		r_obj_to_cam = np.eye(3, dtype=np.float64)
	else:
		r_obj_to_cam, _jacobian = cv2.Rodrigues(np.asarray(detection.rvec, dtype=np.float64))

	t_obj_in_cam_tello = opencv_to_tello_vector(t_obj_in_cam).reshape(3, 1)
	r_obj_to_cam_tello = opencv_to_tello_rotation(r_obj_to_cam)
	r_cam_to_obj_tello = r_obj_to_cam_tello.T
	camera_position_obj = (-r_cam_to_obj_tello @ t_obj_in_cam_tello).reshape(3)

	r_world_obj = yaw_rotation_matrix_z(object_yaw_deg)
	camera_position_world = object_position + (r_world_obj @ camera_position_obj)
	r_world_cam = r_world_obj @ r_cam_to_obj_tello
	return CameraWorldPose(
		position_cm=tuple(float(value) for value in camera_position_world),
		rotation_matrix=r_world_cam,
		method=detection.pose_method,
	)
