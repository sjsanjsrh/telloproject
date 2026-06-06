from __future__ import annotations

import ctypes
import ctypes.util
import math

import cv2
import numpy as np

from yolo_seg.scene_map import Obstacle, SceneMap
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
		return np.array([view[1], view[2], view[0]], dtype=np.float64)

	def _project(self, point_cm) -> tuple[int, int]:
		x, y, _z = self._view_point(point_cm)
		screen_x = x * self.scale + self.width * 0.5
		screen_y = -y * self.scale + self.height * 0.58
		return int(round(screen_x)), int(round(screen_y))

	def _draw_line_3d(self, image: np.ndarray, point_a, point_b, color, thickness: int = 2):
		cv2.line(image, self._project(point_a), self._project(point_b), color, thickness, cv2.LINE_AA)

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

	def _draw_obstacle(self, image: np.ndarray, obstacle: Obstacle, active: bool):
		corners = self._obstacle_corners(obstacle)
		color = (40, 210, 255) if active else (140, 180, 210)
		thickness = 3 if active else 2
		self._draw_column(image, obstacle, color)
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

	def _draw_column(self, image: np.ndarray, obstacle: Obstacle, color):
		center = np.asarray(obstacle.position_cm, dtype=np.float64).reshape(3)
		base = np.array([center[0], center[1], 0.0], dtype=np.float64)
		self._draw_line_3d(image, base, center, (90, 90, 90), 2)
		cv2.circle(image, self._project(base), 4, (90, 90, 90), -1, cv2.LINE_AA)
		cv2.putText(
			image,
			f"{center[2]:.0f}cm",
			(self._project(center)[0] + 8, self._project(center)[1] + 14),
			cv2.FONT_HERSHEY_SIMPLEX,
			0.45,
			color,
			1,
			cv2.LINE_AA,
		)

	def _draw_camera_pose(self, image: np.ndarray, camera_pose: CameraWorldPose | None):
		if camera_pose is None:
			return

		position = np.asarray(camera_pose.position_cm, dtype=np.float64)
		center_2d = self._project(position)
		cv2.circle(image, center_2d, 7, (0, 255, 120), -1, cv2.LINE_AA)
		cv2.putText(
			image,
			f"drone ({position[0]:.0f},{position[1]:.0f},{position[2]:.0f})",
			(center_2d[0] + 10, center_2d[1] - 10),
			cv2.FONT_HERSHEY_SIMPLEX,
			0.52,
			(0, 255, 120),
			2,
			cv2.LINE_AA,
		)
		yaw_deg, pitch_deg, roll_deg = self._camera_angles(camera_pose)
		cv2.putText(
			image,
			f"yaw {yaw_deg:.0f} pitch {pitch_deg:.0f} roll {roll_deg:.0f}",
			(center_2d[0] + 10, center_2d[1] + 12),
			cv2.FONT_HERSHEY_SIMPLEX,
			0.48,
			(0, 255, 120),
			1,
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
			f"tello tvec ({position[0]:.0f},{position[1]:.0f},{position[2]:.0f})",
			(point_2d[0] + 10, point_2d[1] + 18),
			cv2.FONT_HERSHEY_SIMPLEX,
			0.5,
			(255, 80, 220),
			2,
			cv2.LINE_AA,
		)

	def render(
		self,
		scene_map: SceneMap | None,
		active_obstacle: Obstacle | None = None,
		camera_pose: CameraWorldPose | None = None,
		camera_local_detection_cm=None,
	) -> np.ndarray:
		image = np.zeros((self.height, self.width, 3), dtype=np.uint8)
		image[:] = (20, 22, 24)
		self._draw_grid(image)

		if scene_map is not None:
			for obstacle in scene_map.obstacles.values():
				self._draw_obstacle(image, obstacle, active=active_obstacle is not None and obstacle.id == active_obstacle.id)

		self._draw_camera_pose(image, camera_pose)
		self._draw_camera_local_detection(image, camera_local_detection_cm)
		cv2.putText(image, "3D scene map", (18, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (235, 235, 235), 2, cv2.LINE_AA)
		return image

	def show(
		self,
		scene_map: SceneMap | None,
		active_obstacle: Obstacle | None = None,
		camera_pose: CameraWorldPose | None = None,
		camera_local_detection_cm=None,
	):
		cv2.imshow(self.window_name, self.render(scene_map, active_obstacle, camera_pose, camera_local_detection_cm))

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
			self.scale = min(6.0, self.scale + 0.25)
		elif key in (ord("x"), ord("X")):
			self.scale = max(0.5, self.scale - 0.25)
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
		gl.glMatrixMode(GL_PROJECTION)
		gl.glLoadIdentity()
		aspect = self.width / float(self.height)
		view_size = 360.0
		gl.glOrtho(-view_size * aspect, view_size * aspect, -view_size, view_size, -1200.0, 1200.0)

	def _set_camera(self):
		gl = self._gl
		gl.glMatrixMode(GL_MODELVIEW)
		gl.glLoadIdentity()
		gl.glTranslated(0.0, 0.0, -self.distance_cm)
		gl.glRotated(self.pitch_deg, 1.0, 0.0, 0.0)
		gl.glRotated(self.yaw_deg, 0.0, 1.0, 0.0)
		gl.glTranslated(-80.0, -60.0, 100.0)

	def _color(self, bgr):
		b, g, r = bgr
		self._gl.glColor3f(float(r) / 255.0, float(g) / 255.0, float(b) / 255.0)

	def _vertex(self, point):
		x, y, z = np.asarray(point, dtype=np.float64).reshape(3)
		self._gl.glVertex3d(float(y), float(z), float(-x))

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

	def _draw_obstacle(self, obstacle: Obstacle, active: bool):
		gl = self._gl
		corners = self._obstacle_corners(obstacle)
		color = (40, 210, 255) if active else (140, 180, 210)
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
		gl.glRasterPos3d(float(y), float(z), float(-x))
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
		self._label(f"drone {position[0]:.0f},{position[1]:.0f},{position[2]:.0f}", position + np.array([10.0, 12.0, 0.0]), (0, 255, 120))
		yaw_deg, pitch_deg, roll_deg = self._camera_angles(camera_pose)
		self._label(f"yaw {yaw_deg:.0f} pitch {pitch_deg:.0f} roll {roll_deg:.0f}", position + np.array([10.0, 20.0, 0.0]), (0, 255, 120))
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
		self._label(f"tello {position[0]:.0f},{position[1]:.0f},{position[2]:.0f}", position + np.array([8.0, 8.0, 0.0]), (255, 80, 220))

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
			self.distance_cm = max(160.0, self.distance_cm - 35.0)
		elif key in (ord("x"), ord("X")):
			self.distance_cm += 35.0
		elif key in (ord("r"), ord("R")):
			self.yaw_deg = -35.0
			self.pitch_deg = 28.0
			self.distance_cm = 520.0

	def show(
		self,
		scene_map: SceneMap | None,
		active_obstacle: Obstacle | None = None,
		camera_pose: CameraWorldPose | None = None,
		camera_local_detection_cm=None,
	):
		for event in self._pygame.event.get():
			if event.type == self._pygame.QUIT:
				return
			if event.type == self._pygame.KEYDOWN:
				self.handle_key(event.key)

		gl = self._gl
		gl.glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
		self._set_camera()
		self._draw_grid()
		if scene_map is not None:
			for obstacle in scene_map.obstacles.values():
				self._draw_obstacle(obstacle, active=active_obstacle is not None and obstacle.id == active_obstacle.id)
		self._draw_camera_pose(camera_pose)
		self._draw_camera_local_detection(camera_local_detection_cm)
		self._pygame.display.flip()
