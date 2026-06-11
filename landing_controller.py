from __future__ import annotations

import math
import time
from dataclasses import dataclass

import cv2
import numpy as np

from camera_calibration.camera_tranceform import CameraTransform
from pid_controller import DronePIDController
from telloController import TelloController


@dataclass
class LandingConfig:
	camera_params_path: str = "camera_calibration/camera_params.yaml"
	white_threshold: int = 200
	white_threshold_contour: int = 150
	ellipse_area_ratio_range: tuple[float, float] = (0.85, 1.15)
	ellipse_aspect_ratio_threshold: float = 1.1
	ellipse_size_threshold: float = 50.0
	target_error_threshold: float = 0.1
	target_error_threshold_contour: float = 0.07
	contour_height_cm: int = 60
	hold_time_sec: float = 0.4
	descent_step_cm: int = 50
	descent_speed: int = 100
	window_name: str = "show_frame"


class LandingController:
	"""Downward-camera landing target tracker and PID landing loop."""

	def __init__(self, tello, config: LandingConfig | None = None):
		self.tello = tello
		self.config = config or LandingConfig()
		self.camera = CameraTransform(
			self.config.camera_params_path,
			camera_direction="downward",
		)
		self.target_point: tuple[int, int] | None = None
		self.tracking_mode = "init"

	@staticmethod
	def ellipse_aspect_ratio(ellipse) -> float:
		major_axis = max(ellipse[1][0], ellipse[1][1])
		minor_axis = min(ellipse[1][0], ellipse[1][1])
		return major_axis / minor_axis if minor_axis > 0 else 0.0

	def setup_video(self, show_video: bool = True) -> None:
		self.tello.setUpVideo(
			show_video=show_video,
			camera_direction=TelloController.CAMERA_DOWNWARD,
			frame_callback=self.frame_callback,
		)

	def frame_callback(self, frame) -> None:
		target = None
		white_threshold = self.config.white_threshold
		if self.tracking_mode == "contour":
			white_threshold = self.config.white_threshold_contour
		if frame is None:
			return

		gray = frame[:, :, 0]
		_, binary_frame = cv2.threshold(gray, white_threshold, 255, cv2.THRESH_BINARY)
		show_frame = np.copy(frame)
		contours, _hierarchy = cv2.findContours(binary_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		ellipses = []

		if self.tracking_mode == "ellipse":
			for contour in contours:
				if len(contour) < 5:
					continue
				ellipse = cv2.fitEllipse(contour)
				contour_area = cv2.contourArea(contour)
				ellipse_area = np.pi * (ellipse[1][0] / 2.0) * (ellipse[1][1] / 2.0)
				area_ratio = contour_area / ellipse_area if ellipse_area > 0 else 0.0
				if (
					self.config.ellipse_area_ratio_range[0] < area_ratio < self.config.ellipse_area_ratio_range[1]
					and self.ellipse_aspect_ratio(ellipse) < self.config.ellipse_aspect_ratio_threshold
					and contour_area > self.config.ellipse_size_threshold
				):
					ellipses.append(ellipse)

			self.target_point = None
			if ellipses:
				target = max(ellipses, key=lambda item: item[1][0] * item[1][1])
		elif self.tracking_mode == "contour":
			target = max(contours, key=cv2.contourArea, default=None)

		if target is not None:
			if self.tracking_mode == "ellipse":
				self.target_point = (int(target[0][0]), int(target[0][1]))
			elif self.tracking_mode == "contour":
				moments = cv2.moments(target)
				if moments["m00"] != 0:
					self.target_point = (int(moments["m10"] / moments["m00"]), int(moments["m01"] / moments["m00"]))

			if self.target_point is not None:
				cv2.circle(show_frame, self.target_point, 5, (0, 0, 255), -1)

		for ellipse in ellipses:
			cv2.ellipse(show_frame, ellipse, (0, 255, 0), 2)
		cv2.drawContours(show_frame, contours, -1, (255, 0, 0), 1)
		cv2.putText(show_frame, self.tracking_mode, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
		cv2.imshow(self.config.window_name, show_frame)

	def run(self) -> None:
		print("landing start")
		height = self.tello.get_height()
		print(f"current height: {height}cm")
		self.tracking_mode = "ellipse"
		current_hold_time = self.config.hold_time_sec
		pid = DronePIDController(self.tello)
		pid.init_dt()

		while self.tracking_mode != "land":
			height = max(self.tello.get_height(), self.config.contour_height_cm)
			if height <= self.config.contour_height_cm:
				self.tracking_mode = "contour"
			print(f"current height: {height}cm")

			if self.target_point is None:
				time.sleep(0.001)
				continue

			u, v = self.target_point
			x_cm, y_cm = self.camera.uv_to_cm(u, v, height, undistort=True)
			dt = pid.compute_dt()
			if dt == 0:
				time.sleep(0.01)
				continue

			error = math.sqrt(x_cm**2 + y_cm**2) / height
			threshold = (
				self.config.target_error_threshold
				if self.tracking_mode != "contour"
				else self.config.target_error_threshold_contour
			)
			if error > threshold:
				current_hold_time = self.config.hold_time_sec
				print(f"landing target: ({x_cm:.2f}, {y_cm:.2f}) cm, error={error:.2f}")
				pid.control_position(x_cm, y_cm, dt=dt)
			else:
				current_hold_time -= dt
				print(f"landing target hold: {current_hold_time:.2f}s")

			if current_hold_time <= 0:
				print("landing target reached")
				if self.tracking_mode != "contour":
					self.tello.go_xyz_speed(0, 0, -abs(self.config.descent_step_cm), self.config.descent_speed)
					pid.reset()
					pid.init_dt()
				else:
					self.tello.land()
					pid.reset()
					pid.init_dt()
					self.tracking_mode = "land"

			time.sleep(0.1)
