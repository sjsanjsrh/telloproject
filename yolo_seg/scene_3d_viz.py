from __future__ import annotations

import argparse
import ctypes
import ctypes.util
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
	sys.path.insert(0, str(PROJECT_ROOT))

from yolo_seg.flight_plan import flight_plan_points, load_flight_plan
from yolo_seg.scene_map import Obstacle, SceneMap, load_scene_map
from yolo_seg.world_pose import CameraWorldPose, opencv_to_tello_vector, yaw_rotation_matrix_z


GL_COLOR_BUFFER_BIT = 0x00004000
GL_DEPTH_BUFFER_BIT = 0x00000100
GL_LINES = 0x0001
GL_LINE_LOOP = 0x0002
GL_QUADS = 0x0007
GL_MODELVIEW = 0x1700
GL_PROJECTION = 0x1701
GL_DEPTH_TEST = 0x0B71
GL_RGBA = 0x1908
GL_UNSIGNED_BYTE = 0x1401
GL_UNPACK_ALIGNMENT = 0x0CF5
GL_BLEND = 0x0BE2
GL_SRC_ALPHA = 0x0302
GL_ONE_MINUS_SRC_ALPHA = 0x0303


def create_scene_3d_visualizer(
	width: int = 720,
	height: int = 540,
	scale: float = 2.0,
	window_name: str = "scene_3d",
	camera_fov_deg: tuple[float, float] | None = None,
	prefer_gpu: bool = True,
):
	if prefer_gpu:
		try:
			return OpenGLScene3DVisualizer(
				width=width,
				height=height,
				scale=scale,
				window_name=window_name,
				camera_fov_deg=camera_fov_deg,
			)
		except Exception as exc:
			print(f"OpenGL 3D renderer unavailable, falling back to OpenCV CPU renderer: {exc}")
	return Scene3DVisualizer(
		width=width,
		height=height,
		scale=scale,
		window_name=window_name,
		camera_fov_deg=camera_fov_deg,
	)


class Scene3DVisualizer:
	def __init__(
		self,
		width: int = 720,
		height: int = 540,
		scale: float = 2.0,
		window_name: str = "scene_3d",
		camera_fov_deg: tuple[float, float] | None = None,
	):
		self.width = int(width)
		self.height = int(height)
		self.scale = float(scale)
		self.window_name = window_name
		self.yaw_deg = -35.0
		self.pitch_deg = 28.0
		self.camera_fov_deg = camera_fov_deg or (60.0, 45.0)
		self.camera_frustum_length_cm = 220.0
		self._mouse_callback_set = False

	def _zoom(self, factor: float):
		self.scale = float(np.clip(self.scale * factor, 0.45, 7.5))

	def _on_mouse(self, event, _x, _y, flags, _param):
		if event != cv2.EVENT_MOUSEWHEEL:
			return
		self._zoom(1.12 if flags > 0 else 1.0 / 1.12)

	def _view_point(self, point_cm) -> np.ndarray:
		point = np.asarray(point_cm, dtype=np.float64).reshape(3) - np.array([100.0, 80.0, 60.0], dtype=np.float64)
		yaw = math.radians(self.yaw_deg)
		pitch = math.radians(self.pitch_deg)
		yaw_matrix = np.array(
			[
				[math.cos(yaw), -math.sin(yaw), 0.0],
				[math.sin(yaw), math.cos(yaw), 0.0],
				[0.0, 0.0, 1.0],
			],
			dtype=np.float64,
		)
		pitch_matrix = np.array(
			[
				[math.cos(pitch), 0.0, math.sin(pitch)],
				[0.0, 1.0, 0.0],
				[-math.sin(pitch), 0.0, math.cos(pitch)],
			],
			dtype=np.float64,
		)
		view = pitch_matrix @ yaw_matrix @ point
		return np.array([view[0], view[2], view[1]], dtype=np.float64)

	def _project(self, point_cm) -> tuple[int, int]:
		x, y, _z = self._view_point(point_cm)
		screen_x = x * self.scale + self.width * 0.5
		screen_y = -y * self.scale + self.height * 0.58
		return int(round(screen_x)), int(round(screen_y))

	def _draw_line_3d(self, image: np.ndarray, point_a, point_b, color, thickness: int = 2):
		cv2.line(image, self._project(point_a), self._project(point_b), color, thickness, cv2.LINE_AA)

	def _obstacle_color(self, obstacle: Obstacle, active: bool) -> tuple[int, int, int]:
		if obstacle.color_bgr is not None:
			return obstacle.color_bgr
		return (40, 210, 255) if active else (140, 180, 210)

	def _ground_center(self, obstacle: Obstacle) -> np.ndarray:
		center = np.asarray(obstacle.position_cm, dtype=np.float64).reshape(3)
		return np.array([center[0], center[1], center[2]], dtype=np.float64)

	def _camera_angles(self, camera_pose: CameraWorldPose) -> tuple[float, float, float]:
		rotation = np.asarray(camera_pose.rotation_matrix, dtype=np.float64).reshape(3, 3)
		forward = rotation @ np.array([1.0, 0.0, 0.0], dtype=np.float64)
		up = rotation @ np.array([0.0, 0.0, 1.0], dtype=np.float64)
		yaw_deg = float(np.degrees(np.arctan2(forward[1], forward[0])))
		horizontal = float(np.hypot(forward[0], forward[1]))
		pitch_deg = float(np.degrees(np.arctan2(forward[2], max(horizontal, 1e-9))))
		roll_deg = float(np.degrees(np.arctan2(up[1], up[2])))
		return yaw_deg, pitch_deg, roll_deg

	def _draw_grid(self, image: np.ndarray):
		grid_color = (48, 48, 48)
		for value in range(-100, 401, 100):
			self._draw_line_3d(image, (value, -100, 0), (value, 400, 0), grid_color, 1)
			self._draw_line_3d(image, (-100, value, 0), (400, value, 0), grid_color, 1)

		self._draw_line_3d(image, (0, 0, 0), (160, 0, 0), (80, 80, 255), 2)
		self._draw_line_3d(image, (0, 0, 0), (0, 160, 0), (80, 255, 80), 2)
		self._draw_line_3d(image, (0, 0, 0), (0, 0, 160), (255, 120, 80), 2)
		cv2.putText(image, "X", self._project((170, 0, 0)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (80, 80, 255), 2)
		cv2.putText(image, "Y", self._project((0, 170, 0)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (80, 255, 80), 2)
		cv2.putText(image, "Z", self._project((0, 0, 170)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 120, 80), 2)

	def _obstacle_corners(self, obstacle: Obstacle) -> np.ndarray:
		half_width = obstacle.shape.width_cm / 2.0
		half_height = obstacle.shape.height_cm / 2.0
		local = np.array(
			[
				[0.0, -half_width, -half_height],
				[0.0, half_width, -half_height],
				[0.0, half_width, half_height],
				[0.0, -half_width, half_height],
			],
			dtype=np.float64,
		)
		rotation = yaw_rotation_matrix_z(obstacle.yaw_deg)
		center = np.asarray(obstacle.position_cm, dtype=np.float64).reshape(3)
		return (rotation @ local.T).T + center

	def _ground_rectangle_corners(self, obstacle: Obstacle) -> np.ndarray:
		half_width = obstacle.shape.width_cm / 2.0
		half_height = obstacle.shape.height_cm / 2.0
		local = np.array(
			[
				[-half_width, -half_height, 0.0],
				[half_width, -half_height, 0.0],
				[half_width, half_height, 0.0],
				[-half_width, half_height, 0.0],
			],
			dtype=np.float64,
		)
		rotation = yaw_rotation_matrix_z(obstacle.yaw_deg)
		center = self._ground_center(obstacle)
		return (rotation @ local.T).T + center

	def _draw_ground_loop(self, image: np.ndarray, center: np.ndarray, radius_x: float, radius_y: float, color, thickness: int = 2, points: int = 48):
		angles = np.linspace(0.0, 2.0 * np.pi, points, endpoint=True)
		loop = [
			center + np.array([math.cos(angle) * radius_x, math.sin(angle) * radius_y, 0.0], dtype=np.float64)
			for angle in angles
		]
		for index in range(len(loop) - 1):
			self._draw_line_3d(image, loop[index], loop[index + 1], color, thickness)

	def _draw_start_area(self, image: np.ndarray, obstacle: Obstacle, active: bool):
		color = self._obstacle_color(obstacle, active)
		corners = self._ground_rectangle_corners(obstacle)
		thickness = 3 if active else 2
		for index in range(4):
			self._draw_line_3d(image, corners[index], corners[(index + 1) % 4], color, thickness)
		center = self._ground_center(obstacle)
		cv2.circle(image, self._project(center), 4, color, -1, cv2.LINE_AA)
		self._draw_column(image, obstacle, color, base_z=float(center[2]), height_cm=0.0)

	def _draw_landing_marker(self, image: np.ndarray, obstacle: Obstacle, active: bool):
		color = self._obstacle_color(obstacle, active)
		center = self._ground_center(obstacle)
		diameter = float(obstacle.shape.diameter_cm or max(obstacle.shape.width_cm, obstacle.shape.height_cm))
		self._draw_ground_loop(image, center, diameter / 2.0, diameter / 2.0, color, thickness=3 if active else 2)
		pillar_height = obstacle.shape.pillar_height_cm
		if pillar_height is None:
			pillar_height = max(10.0, obstacle.shape.thickness_cm * 10.0 if obstacle.shape.thickness_cm > 0.0 else 20.0)
		pillar_radius = obstacle.shape.pillar_diameter_cm or max(4.0, diameter * 0.07)
		base = center
		top = center + np.array([0.0, 0.0, pillar_height], dtype=np.float64)
		self._draw_line_3d(image, base, top, color, 3 if active else 2)
		self._draw_ground_loop(image, base, pillar_radius * 0.5, pillar_radius * 0.5, color, thickness=2)
		cv2.circle(image, self._project(top), 4, color, -1, cv2.LINE_AA)

	def _draw_shape(self, image: np.ndarray, obstacle: Obstacle, active: bool):
		shape_type = obstacle.shape.type.lower()
		if shape_type in {"start_area", "start_rectangle"}:
			self._draw_start_area(image, obstacle, active)
			return
		if shape_type in {"landing_point", "landing_zone", "landing_marker", "circle_pillar"}:
			self._draw_landing_marker(image, obstacle, active)
			return

		color = self._obstacle_color(obstacle, active)
		corners = self._obstacle_corners(obstacle)
		thickness = 3 if active else 2
		self._draw_column(image, obstacle, color, base_z=0.0)
		for index in range(4):
			self._draw_line_3d(image, corners[index], corners[(index + 1) % 4], color, thickness)
		center_2d = self._project(obstacle.position_cm)
		cv2.circle(image, center_2d, 4, color, -1, cv2.LINE_AA)
		cv2.putText(
			image,
			f"{obstacle.id}",
			(center_2d[0] + 8, center_2d[1] - 8),
			cv2.FONT_HERSHEY_SIMPLEX,
			0.5,
			color,
			1,
			cv2.LINE_AA,
		)

	def _draw_column(self, image: np.ndarray, obstacle: Obstacle, color, base_z: float = 0.0, height_cm: float | None = None):
		center = np.asarray(obstacle.position_cm, dtype=np.float64).reshape(3)
		base = np.array([center[0], center[1], base_z], dtype=np.float64)
		top = center if height_cm is None else np.array([center[0], center[1], base_z + height_cm], dtype=np.float64)
		self._draw_line_3d(image, base, top, (90, 90, 90), 2)
		cv2.circle(image, self._project(base), 4, (90, 90, 90), -1, cv2.LINE_AA)
		if height_cm is None:
			return

	def _draw_camera_pose(self, image: np.ndarray, camera_pose: CameraWorldPose | None):
		if camera_pose is None:
			return

		position = np.asarray(camera_pose.position_cm, dtype=np.float64)
		center_2d = self._project(position)
		cv2.circle(image, center_2d, 7, (0, 255, 120), -1, cv2.LINE_AA)
		cv2.putText(
			image,
			"drone",
			(center_2d[0] + 10, center_2d[1] - 10),
			cv2.FONT_HERSHEY_SIMPLEX,
			0.52,
			(0, 255, 120),
			2,
			cv2.LINE_AA,
		)

		for axis, color in [
			(np.array([35.0, 0.0, 0.0]), (80, 80, 255)),
			(np.array([0.0, 35.0, 0.0]), (80, 255, 80)),
			(np.array([0.0, 0.0, 35.0]), (255, 120, 80)),
		]:
			end = position + camera_pose.rotation_matrix @ axis
			self._draw_line_3d(image, position, end, color, 2)
		self._draw_line_3d(image, position, position + camera_pose.rotation_matrix @ np.array([80.0, 0.0, 0.0]), (255, 255, 100), 4)
		self._draw_camera_frustum(image, camera_pose)

	def _draw_camera_frustum(self, image: np.ndarray, camera_pose: CameraWorldPose):
		hfov_rad = math.radians(float(self.camera_fov_deg[0]))
		vfov_rad = math.radians(float(self.camera_fov_deg[1]))
		length = float(self.camera_frustum_length_cm)
		half_w = length * math.tan(hfov_rad * 0.5)
		half_h = length * math.tan(vfov_rad * 0.5)
		local_corners = [
			np.array([length, -half_w, -half_h], dtype=np.float64),
			np.array([length, half_w, -half_h], dtype=np.float64),
			np.array([length, half_w, half_h], dtype=np.float64),
			np.array([length, -half_w, half_h], dtype=np.float64),
		]
		position = np.asarray(camera_pose.position_cm, dtype=np.float64)
		world_corners = [position + camera_pose.rotation_matrix @ corner for corner in local_corners]
		frustum_color = (255, 240, 90)
		for corner in world_corners:
			self._draw_line_3d(image, position, corner, frustum_color, 1)
		for index in range(4):
			self._draw_line_3d(image, world_corners[index], world_corners[(index + 1) % 4], frustum_color, 1)

	def _draw_camera_local_detection(self, image: np.ndarray, detection_position_cm):
		if detection_position_cm is None:
			return

		position = opencv_to_tello_vector(detection_position_cm)
		origin = np.zeros(3, dtype=np.float64)
		point_2d = self._project(position)
		origin_2d = self._project(origin)
		cv2.line(image, origin_2d, point_2d, (255, 80, 220), 2, cv2.LINE_AA)
		cv2.circle(image, point_2d, 6, (255, 80, 220), -1, cv2.LINE_AA)
		cv2.putText(
			image,
			"target",
			(point_2d[0] + 10, point_2d[1] + 18),
			cv2.FONT_HERSHEY_SIMPLEX,
			0.5,
			(255, 80, 220),
			2,
			cv2.LINE_AA,
		)

	def _draw_pose_markers(self, image: np.ndarray, pose_markers):
		if not pose_markers:
			return
		for marker in pose_markers:
			position = np.asarray(marker["position_cm"], dtype=np.float64).reshape(3)
			color = tuple(int(value) for value in marker.get("color_bgr", (255, 255, 255)))
			label = str(marker.get("label", "pose"))
			center_2d = self._project(position)
			cv2.circle(image, center_2d, 7, color, -1, cv2.LINE_AA)
			cv2.putText(
				image,
				label,
				(center_2d[0] + 9, center_2d[1] + 6),
				cv2.FONT_HERSHEY_SIMPLEX,
				0.5,
				color,
				2,
				cv2.LINE_AA,
			)

	def _draw_flight_path(self, image: np.ndarray, flight_path_points):
		if not flight_path_points:
			return

		path_color = (40, 170, 255)
		start_color = (80, 255, 140)
		takeoff_color = (120, 220, 255)
		waypoint_color = (255, 220, 80)
		positions = [np.asarray(point["position_cm"], dtype=np.float64).reshape(3) for point in flight_path_points]
		for index in range(len(positions) - 1):
			self._draw_line_3d(image, positions[index], positions[index + 1], path_color, 2)
		for index, point in enumerate(flight_path_points):
			position = positions[index]
			color = {"start": start_color, "takeoff": takeoff_color}.get(point.get("kind"), waypoint_color)
			center_2d = self._project(position)
			cv2.circle(image, center_2d, 5, color, -1, cv2.LINE_AA)
			cv2.putText(
				image,
				str(point.get("name", index)),
				(center_2d[0] + 8, center_2d[1] + 5),
				cv2.FONT_HERSHEY_SIMPLEX,
				0.42,
				color,
				1,
				cv2.LINE_AA,
			)

	def render(
		self,
		scene_map: SceneMap | None,
		active_obstacle: Obstacle | None = None,
		camera_pose: CameraWorldPose | None = None,
		camera_local_detection_cm=None,
		flight_path_points=None,
		pose_markers=None,
	) -> np.ndarray:
		image = np.zeros((self.height, self.width, 3), dtype=np.uint8)
		image[:] = (20, 22, 24)
		self._draw_grid(image)

		if scene_map is not None:
			for obstacle in scene_map.obstacles.values():
				self._draw_shape(image, obstacle, active=active_obstacle is not None and obstacle.id == active_obstacle.id)

		self._draw_flight_path(image, flight_path_points)
		self._draw_pose_markers(image, pose_markers)
		self._draw_camera_pose(image, camera_pose)
		self._draw_camera_local_detection(image, camera_local_detection_cm)
		return image

	def show(
		self,
		scene_map: SceneMap | None,
		active_obstacle: Obstacle | None = None,
		camera_pose: CameraWorldPose | None = None,
		camera_local_detection_cm=None,
		flight_path_points=None,
		pose_markers=None,
	):
		if not self._mouse_callback_set:
			cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
			cv2.setMouseCallback(self.window_name, self._on_mouse)
			self._mouse_callback_set = True
		cv2.imshow(self.window_name, self.render(scene_map, active_obstacle, camera_pose, camera_local_detection_cm, flight_path_points, pose_markers))
		return True

	def handle_key(self, key: int):
		if key in (ord("a"), ord("A")):
			self.yaw_deg -= 6.0
		elif key in (ord("d"), ord("D")):
			self.yaw_deg += 6.0
		elif key in (ord("w"), ord("W")):
			self.pitch_deg += 4.0
		elif key in (ord("s"), ord("S")):
			self.pitch_deg -= 4.0
		elif key in (ord("z"), ord("Z")):
			self._zoom(1.12)
		elif key in (ord("x"), ord("X")):
			self._zoom(1.0 / 1.12)
		elif key in (ord("r"), ord("R")):
			self.yaw_deg = -35.0
			self.pitch_deg = 28.0
			self.scale = 2.0


class OpenGLScene3DVisualizer:
	def __init__(
		self,
		width: int = 720,
		height: int = 540,
		scale: float = 2.0,
		window_name: str = "scene_3d",
		camera_fov_deg: tuple[float, float] | None = None,
	):
		import pygame

		self.width = int(width)
		self.height = int(height)
		self.scale = float(scale)
		self.window_name = window_name
		self.yaw_deg = -35.0
		self.pitch_deg = 28.0
		self.distance_cm = 520.0
		self.view_size = 360.0 / max(0.1, self.scale)
		self.camera_fov_deg = camera_fov_deg or (60.0, 45.0)
		self.camera_frustum_length_cm = 220.0
		self._pygame = pygame
		self._pygame.init()
		self._font = self._pygame.font.Font(None, 22)
		self._pygame.display.set_caption(window_name)
		self._pygame.display.set_mode((self.width, self.height), self._pygame.OPENGL | self._pygame.DOUBLEBUF)
		self._gl = self._load_gl()
		self._setup_gl()

	def _load_gl(self):
		lib_name = ctypes.util.find_library("opengl32") or ctypes.util.find_library("GL")
		if lib_name:
			gl = ctypes.WinDLL(lib_name) if "opengl32" in lib_name.lower() else ctypes.CDLL(lib_name)
		else:
			try:
				gl = ctypes.WinDLL("opengl32.dll")
			except OSError as exc:
				raise RuntimeError("OpenGL library was not found") from exc
		for name, restype, argtypes in [
			("glClearColor", None, [ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float]),
			("glClear", None, [ctypes.c_uint]),
			("glEnable", None, [ctypes.c_uint]),
			("glDisable", None, [ctypes.c_uint]),
			("glViewport", None, [ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int]),
			("glMatrixMode", None, [ctypes.c_uint]),
			("glLoadIdentity", None, []),
			("glOrtho", None, [ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double]),
			("glTranslated", None, [ctypes.c_double, ctypes.c_double, ctypes.c_double]),
			("glRotated", None, [ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double]),
			("glBegin", None, [ctypes.c_uint]),
			("glEnd", None, []),
			("glColor3f", None, [ctypes.c_float, ctypes.c_float, ctypes.c_float]),
			("glVertex3d", None, [ctypes.c_double, ctypes.c_double, ctypes.c_double]),
			("glRasterPos3d", None, [ctypes.c_double, ctypes.c_double, ctypes.c_double]),
			("glDrawPixels", None, [ctypes.c_int, ctypes.c_int, ctypes.c_uint, ctypes.c_uint, ctypes.c_void_p]),
			("glPixelStorei", None, [ctypes.c_uint, ctypes.c_int]),
			("glBlendFunc", None, [ctypes.c_uint, ctypes.c_uint]),
			("glLineWidth", None, [ctypes.c_float]),
			("glPointSize", None, [ctypes.c_float]),
		]:
			func = getattr(gl, name)
			func.restype = restype
			func.argtypes = argtypes
		return gl

	def _setup_gl(self):
		gl = self._gl
		gl.glViewport(0, 0, self.width, self.height)
		gl.glClearColor(0.08, 0.085, 0.095, 1.0)
		gl.glEnable(GL_DEPTH_TEST)
		self._set_projection()

	def _set_projection(self):
		gl = self._gl
		gl.glMatrixMode(GL_PROJECTION)
		gl.glLoadIdentity()
		aspect = self.width / float(self.height)
		view_size = float(self.view_size)
		gl.glOrtho(-view_size * aspect, view_size * aspect, -view_size, view_size, -1200.0, 1200.0)

	def _zoom(self, factor: float):
		self.view_size = float(np.clip(self.view_size / factor, 70.0, 780.0))
		self._set_projection()

	def _set_camera(self):
		gl = self._gl
		gl.glMatrixMode(GL_MODELVIEW)
		gl.glLoadIdentity()
		gl.glTranslated(0.0, 0.0, -self.distance_cm)
		gl.glRotated(self.pitch_deg, 1.0, 0.0, 0.0)
		gl.glRotated(self.yaw_deg, 0.0, 0.0, 1.0)
		gl.glTranslated(-100.0, -80.0, -60.0)

	def _color(self, bgr):
		b, g, r = bgr
		self._gl.glColor3f(float(r) / 255.0, float(g) / 255.0, float(b) / 255.0)

	def _vertex(self, point):
		x, y, z = np.asarray(point, dtype=np.float64).reshape(3)
		self._gl.glVertex3d(float(x), float(y), float(z))

	def _line(self, point_a, point_b, color, width: float = 2.0):
		gl = self._gl
		gl.glLineWidth(float(width))
		self._color(color)
		gl.glBegin(GL_LINES)
		self._vertex(point_a)
		self._vertex(point_b)
		gl.glEnd()

	def _draw_grid(self):
		for value in range(-100, 401, 100):
			self._line((value, -100, 0), (value, 400, 0), (48, 48, 48), 1.0)
			self._line((-100, value, 0), (400, value, 0), (48, 48, 48), 1.0)
		self._line((0, 0, 0), (160, 0, 0), (80, 80, 255), 3.0)
		self._line((0, 0, 0), (0, 160, 0), (80, 255, 80), 3.0)
		self._line((0, 0, 0), (0, 0, 160), (255, 120, 80), 3.0)
		self._label("X", (170, 0, 0), (80, 80, 255))
		self._label("Y", (0, 170, 0), (80, 255, 80))
		self._label("Z", (0, 0, 170), (255, 120, 80))

	def _obstacle_corners(self, obstacle: Obstacle) -> np.ndarray:
		half_width = obstacle.shape.width_cm / 2.0
		half_height = obstacle.shape.height_cm / 2.0
		local = np.array(
			[
				[0.0, -half_width, -half_height],
				[0.0, half_width, -half_height],
				[0.0, half_width, half_height],
				[0.0, -half_width, half_height],
			],
			dtype=np.float64,
		)
		rotation = yaw_rotation_matrix_z(obstacle.yaw_deg)
		center = np.asarray(obstacle.position_cm, dtype=np.float64).reshape(3)
		return (rotation @ local.T).T + center

	def _obstacle_color(self, obstacle: Obstacle, active: bool) -> tuple[int, int, int]:
		if obstacle.color_bgr is not None:
			return obstacle.color_bgr
		return (40, 210, 255) if active else (140, 180, 210)

	def _ground_center(self, obstacle: Obstacle) -> np.ndarray:
		return np.asarray(obstacle.position_cm, dtype=np.float64).reshape(3)

	def _ground_rectangle_corners(self, obstacle: Obstacle) -> np.ndarray:
		half_width = obstacle.shape.width_cm / 2.0
		half_height = obstacle.shape.height_cm / 2.0
		local = np.array(
			[
				[-half_width, -half_height, 0.0],
				[half_width, -half_height, 0.0],
				[half_width, half_height, 0.0],
				[-half_width, half_height, 0.0],
			],
			dtype=np.float64,
		)
		rotation = yaw_rotation_matrix_z(obstacle.yaw_deg)
		center = self._ground_center(obstacle)
		return (rotation @ local.T).T + center

	def _draw_ground_loop(self, center: np.ndarray, radius_x: float, radius_y: float, color, width: float = 2.0, points: int = 48):
		angles = np.linspace(0.0, 2.0 * np.pi, points, endpoint=True)
		loop = [
			center + np.array([math.cos(angle) * radius_x, math.sin(angle) * radius_y, 0.0], dtype=np.float64)
			for angle in angles
		]
		self._color(color)
		self._gl.glLineWidth(float(width))
		self._gl.glBegin(GL_LINE_LOOP)
		for point in loop:
			self._vertex(point)
		self._gl.glEnd()

	def _draw_start_area(self, obstacle: Obstacle, active: bool):
		color = self._obstacle_color(obstacle, active)
		corners = self._ground_rectangle_corners(obstacle)
		self._color(color)
		self._gl.glLineWidth(3.0 if active else 2.0)
		self._gl.glBegin(GL_LINE_LOOP)
		for corner in corners:
			self._vertex(corner)
		self._gl.glEnd()
		center = self._ground_center(obstacle)
		self._draw_cube(center, 3.5, color)

	def _draw_landing_marker(self, obstacle: Obstacle, active: bool):
		color = self._obstacle_color(obstacle, active)
		center = self._ground_center(obstacle)
		diameter = float(obstacle.shape.diameter_cm or max(obstacle.shape.width_cm, obstacle.shape.height_cm))
		self._draw_ground_loop(center, diameter / 2.0, diameter / 2.0, color, width=3.0 if active else 2.0)
		pillar_height = obstacle.shape.pillar_height_cm
		if pillar_height is None:
			pillar_height = max(10.0, obstacle.shape.thickness_cm * 10.0 if obstacle.shape.thickness_cm > 0.0 else 20.0)
		top = center + np.array([0.0, 0.0, pillar_height], dtype=np.float64)
		self._line(center, top, color, 3.0 if active else 2.0)
		self._draw_cube(top, 3.5, color)
		self._draw_ground_loop(center, (obstacle.shape.pillar_diameter_cm or max(4.0, diameter * 0.07)) * 0.5, (obstacle.shape.pillar_diameter_cm or max(4.0, diameter * 0.07)) * 0.5, color, width=1.5)

	def _draw_shape(self, obstacle: Obstacle, active: bool):
		shape_type = obstacle.shape.type.lower()
		if shape_type in {"start_area", "start_rectangle"}:
			self._draw_start_area(obstacle, active)
			return
		if shape_type in {"landing_point", "landing_zone", "landing_marker", "circle_pillar"}:
			self._draw_landing_marker(obstacle, active)
			return

		gl = self._gl
		corners = self._obstacle_corners(obstacle)
		color = self._obstacle_color(obstacle, active)
		self._draw_column(obstacle, color)
		self._color(color)
		gl.glLineWidth(4.0 if active else 2.0)
		gl.glBegin(GL_LINE_LOOP)
		for corner in corners:
			self._vertex(corner)
		gl.glEnd()
		center = np.asarray(obstacle.position_cm, dtype=np.float64).reshape(3)
		self._draw_cube(center, 4.0, color)
		self._label(f"{obstacle.id}", center + np.array([8.0, 10.0, 8.0]), color)

	def _draw_column(self, obstacle: Obstacle, color):
		center = np.asarray(obstacle.position_cm, dtype=np.float64).reshape(3)
		base = np.array([center[0], center[1], 0.0], dtype=np.float64)
		self._line(base, center, (90, 90, 90), 2.0)
		self._draw_cube(base, 4.0, (90, 90, 90))
		self._line(base + np.array([-10.0, 0.0, 0.0]), base + np.array([10.0, 0.0, 0.0]), color, 1.5)
		self._line(base + np.array([0.0, -10.0, 0.0]), base + np.array([0.0, 10.0, 0.0]), color, 1.5)

	def _label(self, text: str, point_cm, color):
		surface = self._font.render(text, True, (int(color[2]), int(color[1]), int(color[0]))).convert_alpha()
		width, height = surface.get_size()
		if width <= 0 or height <= 0:
			return
		data = self._pygame.image.tostring(surface, "RGBA", True)
		buffer = ctypes.create_string_buffer(data)
		x, y, z = np.asarray(point_cm, dtype=np.float64).reshape(3)
		gl = self._gl
		gl.glDisable(GL_DEPTH_TEST)
		gl.glEnable(GL_BLEND)
		gl.glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
		gl.glPixelStorei(GL_UNPACK_ALIGNMENT, 1)
		gl.glRasterPos3d(float(x), float(y), float(z))
		gl.glDrawPixels(width, height, GL_RGBA, GL_UNSIGNED_BYTE, buffer)
		gl.glDisable(GL_BLEND)
		gl.glEnable(GL_DEPTH_TEST)

	def _draw_camera_pose(self, camera_pose: CameraWorldPose | None):
		if camera_pose is None:
			return
		position = np.asarray(camera_pose.position_cm, dtype=np.float64)
		self._line(position, position + camera_pose.rotation_matrix @ np.array([45.0, 0.0, 0.0]), (80, 80, 255), 4.0)
		self._line(position, position + camera_pose.rotation_matrix @ np.array([0.0, 45.0, 0.0]), (80, 255, 80), 4.0)
		self._line(position, position + camera_pose.rotation_matrix @ np.array([0.0, 0.0, 45.0]), (255, 120, 80), 4.0)
		self._draw_cube(position, 8.0, (0, 255, 120))
		self._line(position, position + camera_pose.rotation_matrix @ np.array([80.0, 0.0, 0.0]), (255, 255, 100), 4.0)
		self._label("drone", position + np.array([10.0, 12.0, 0.0]), (0, 255, 120))
		self._draw_camera_frustum(camera_pose)

	def _camera_angles(self, camera_pose: CameraWorldPose) -> tuple[float, float, float]:
		rotation = np.asarray(camera_pose.rotation_matrix, dtype=np.float64).reshape(3, 3)
		forward = rotation @ np.array([1.0, 0.0, 0.0], dtype=np.float64)
		up = rotation @ np.array([0.0, 0.0, 1.0], dtype=np.float64)
		yaw_deg = float(np.degrees(np.arctan2(forward[1], forward[0])))
		horizontal = float(np.hypot(forward[0], forward[1]))
		pitch_deg = float(np.degrees(np.arctan2(forward[2], max(horizontal, 1e-9))))
		roll_deg = float(np.degrees(np.arctan2(up[1], up[2])))
		return yaw_deg, pitch_deg, roll_deg

	def _draw_camera_frustum(self, camera_pose: CameraWorldPose):
		hfov_rad = math.radians(float(self.camera_fov_deg[0]))
		vfov_rad = math.radians(float(self.camera_fov_deg[1]))
		length = float(self.camera_frustum_length_cm)
		half_w = length * math.tan(hfov_rad * 0.5)
		half_h = length * math.tan(vfov_rad * 0.5)
		local_corners = [
			np.array([length, -half_w, -half_h], dtype=np.float64),
			np.array([length, half_w, -half_h], dtype=np.float64),
			np.array([length, half_w, half_h], dtype=np.float64),
			np.array([length, -half_w, half_h], dtype=np.float64),
		]
		position = np.asarray(camera_pose.position_cm, dtype=np.float64)
		world_corners = [position + camera_pose.rotation_matrix @ corner for corner in local_corners]
		frustum_color = (255, 240, 90)
		for corner in world_corners:
			self._line(position, corner, frustum_color, 3.0)
		for index in range(4):
			self._line(world_corners[index], world_corners[(index + 1) % 4], frustum_color, 3.0)

	def _draw_camera_local_detection(self, detection_position_cm):
		if detection_position_cm is None:
			return
		position = opencv_to_tello_vector(detection_position_cm)
		self._line((0, 0, 0), position, (255, 80, 220), 3.0)
		self._draw_cube(position, 6.0, (255, 80, 220))
		self._label("target", position + np.array([8.0, 8.0, 0.0]), (255, 80, 220))

	def _draw_pose_markers(self, pose_markers):
		if not pose_markers:
			return
		for marker in pose_markers:
			position = np.asarray(marker["position_cm"], dtype=np.float64).reshape(3)
			color = tuple(int(value) for value in marker.get("color_bgr", (255, 255, 255)))
			label = str(marker.get("label", "pose"))
			self._draw_cube(position, 7.0, color)
			self._label(label, position + np.array([8.0, 8.0, 8.0], dtype=np.float64), color)

	def _draw_flight_path(self, flight_path_points):
		if not flight_path_points:
			return

		path_color = (40, 170, 255)
		start_color = (80, 255, 140)
		takeoff_color = (120, 220, 255)
		waypoint_color = (255, 220, 80)
		positions = [np.asarray(point["position_cm"], dtype=np.float64).reshape(3) for point in flight_path_points]
		for index in range(len(positions) - 1):
			self._line(positions[index], positions[index + 1], path_color, 3.0)
		for index, point in enumerate(flight_path_points):
			position = positions[index]
			color = {"start": start_color, "takeoff": takeoff_color}.get(point.get("kind"), waypoint_color)
			self._draw_cube(position, 4.0, color)
			self._label(str(point.get("name", index)), position + np.array([8.0, 8.0, 5.0]), color)

	def _draw_cube(self, center, radius: float, color):
		gl = self._gl
		center = np.asarray(center, dtype=np.float64).reshape(3)
		r = float(radius)
		points = [
			center + np.array([sx * r, sy * r, sz * r], dtype=np.float64)
			for sx in (-1, 1)
			for sy in (-1, 1)
			for sz in (-1, 1)
		]
		edges = [(0, 1), (0, 2), (0, 4), (3, 1), (3, 2), (3, 7), (5, 1), (5, 4), (5, 7), (6, 2), (6, 4), (6, 7)]
		self._color(color)
		gl.glLineWidth(2.0)
		gl.glBegin(GL_LINES)
		for a, b in edges:
			self._vertex(points[a])
			self._vertex(points[b])
		gl.glEnd()

	def handle_key(self, key: int):
		if key in (ord("a"), ord("A")):
			self.yaw_deg -= 6.0
		elif key in (ord("d"), ord("D")):
			self.yaw_deg += 6.0
		elif key in (ord("w"), ord("W")):
			self.pitch_deg += 4.0
		elif key in (ord("s"), ord("S")):
			self.pitch_deg -= 4.0
		elif key in (ord("z"), ord("Z")):
			self._zoom(1.12)
		elif key in (ord("x"), ord("X")):
			self._zoom(1.0 / 1.12)
		elif key in (ord("r"), ord("R")):
			self.yaw_deg = -35.0
			self.pitch_deg = 28.0
			self.distance_cm = 520.0
			self.view_size = 360.0 / max(0.1, self.scale)
			self._set_projection()

	def show(
		self,
		scene_map: SceneMap | None,
		active_obstacle: Obstacle | None = None,
		camera_pose: CameraWorldPose | None = None,
		camera_local_detection_cm=None,
		flight_path_points=None,
		pose_markers=None,
	):
		for event in self._pygame.event.get():
			if event.type == self._pygame.QUIT:
				return False
			if event.type == self._pygame.KEYDOWN:
				self.handle_key(event.key)
			if event.type == self._pygame.MOUSEWHEEL:
				self._zoom(1.12 if event.y > 0 else 1.0 / 1.12)

		gl = self._gl
		gl.glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
		self._set_camera()
		self._draw_grid()
		if scene_map is not None:
			for obstacle in scene_map.obstacles.values():
				self._draw_shape(obstacle, active=active_obstacle is not None and obstacle.id == active_obstacle.id)
		self._draw_flight_path(flight_path_points)
		self._draw_pose_markers(pose_markers)
		self._draw_camera_pose(camera_pose)
		self._draw_camera_local_detection(camera_local_detection_cm)
		self._pygame.display.flip()
		return True


def parse_viewer_args() -> argparse.Namespace:
	parser = argparse.ArgumentParser(description="Standalone 3D scene and flight path viewer")
	parser.add_argument("--scene-map", default=str(Path("yolo_seg") / "obstacles.yaml"), help="Scene obstacle YAML path")
	parser.add_argument("--flight-plan", default="flight_path.yaml", help="Flight path YAML path")
	parser.add_argument("--renderer", default="gpu", choices=("gpu", "cpu"), help="3D renderer backend")
	parser.add_argument("--width", type=int, default=720, help="Viewer window width")
	parser.add_argument("--height", type=int, default=540, help="Viewer window height")
	return parser.parse_args()


def load_optional_scene_map(path: str | Path) -> SceneMap | None:
	scene_path = Path(path)
	if not path or not scene_path.exists():
		return None
	return load_scene_map(scene_path)


def main() -> None:
	args = parse_viewer_args()
	scene_map = load_optional_scene_map(args.scene_map)
	points = flight_plan_points(load_flight_plan(args.flight_plan))
	viewer = create_scene_3d_visualizer(
		width=args.width,
		height=args.height,
		prefer_gpu=args.renderer == "gpu",
	)

	while True:
		keep_open = viewer.show(scene_map, flight_path_points=points)
		if keep_open is False:
			break
		key = cv2.waitKey(16) & 0xFF
		if hasattr(viewer, "handle_key"):
			viewer.handle_key(key)
		if key in (27, ord("q")):
			break
		time.sleep(0.01)

	cv2.destroyAllWindows()


if __name__ == "__main__":
	main()
