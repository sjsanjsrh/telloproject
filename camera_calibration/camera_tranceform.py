import cv2
import numpy as np
import yaml


CAMERA_ALIASES = {
    "front": "forward",
    "forward": "forward",
    "f": "forward",
    "0": "forward",
    "down": "downward",
    "bottom": "downward",
    "downward": "downward",
    "d": "downward",
    "1": "downward",
}


def normalize_camera(camera):
    if camera is None:
        return None
    return CAMERA_ALIASES.get(str(camera).lower(), str(camera).lower())


def make_profile_name(camera_direction=None, resolution=None, camera_profile=None):
    if camera_profile:
        return str(camera_profile)
    camera = normalize_camera(camera_direction)
    if camera is None:
        return None
    if camera == "downward":
        return "downward"
    if resolution is None:
        return None
    return f"{camera}_{int(resolution)}p"


def select_camera_params(params, camera_profile=None, camera_direction=None, resolution=None):
    profiles = params.get("profiles")
    if not profiles:
        return params

    selected_profile = make_profile_name(
        camera_direction=camera_direction,
        resolution=resolution,
        camera_profile=camera_profile,
    )
    if selected_profile is None:
        selected_profile = params.get("default_profile")
    if selected_profile in profiles:
        return profiles[selected_profile]

    available = ", ".join(sorted(profiles))
    raise KeyError(f"Camera calibration profile not found: {selected_profile}. Available: {available}")


def load_camera_params(yaml_path, camera_profile=None, camera_direction=None, resolution=None):
    with open(yaml_path, "r", encoding="utf-8") as file_handle:
        params = yaml.safe_load(file_handle) or {}
    params = select_camera_params(
        params,
        camera_profile=camera_profile,
        camera_direction=camera_direction,
        resolution=resolution,
    )
    camera_matrix = np.array(params["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
    dist_coeffs = np.array(params["distortion_coefficients"]["data"], dtype=np.float64)
    return camera_matrix, dist_coeffs


class CameraTransform:
    def __init__(self, yaml_path, camera_profile=None, camera_direction=None, resolution=None):
        with open(yaml_path, "r", encoding="utf-8") as f:
            params = yaml.safe_load(f) or {}
        params = select_camera_params(
            params,
            camera_profile=camera_profile,
            camera_direction=camera_direction,
            resolution=resolution,
        )
        self.K = np.array(params["camera_matrix"]["data"]).reshape(3, 3)
        self.dist = np.array(params["distortion_coefficients"]["data"])
        self.image_width = params.get("image_width")
        self.image_height = params.get("image_height")
        self.camera = params.get("camera")
        self.resolution = params.get("resolution")
        self.fx = self.K[0, 0]
        self.fy = self.K[1, 1]
        self.cx = self.K[0, 2]
        self.cy = self.K[1, 2]

    def uv_to_cm(self, u, v, Z_cm, undistort=False, camera_pitch_deg=0, camera_roll_deg=0):
        if undistort:
            pts = np.array([[[u, v]]], dtype=np.float32)
            undistorted = cv2.undistortPoints(pts, self.K, self.dist, P=self.K)
            u, v = undistorted[0, 0]
        x = (float(u) - self.cx) / self.fx
        y = (float(v) - self.cy) / self.fy
        theta = np.deg2rad(camera_pitch_deg)
        phi = np.deg2rad(camera_roll_deg)

        Z = float(Z_cm)
        Xc = x * Z
        Yc = y * Z
        vec = np.array([Xc, Yc, Z])

        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(theta), -np.sin(theta)],
            [0, np.sin(theta), np.cos(theta)],
        ])
        Ry = np.array([
            [np.cos(phi), 0, np.sin(phi)],
            [0, 1, 0],
            [-np.sin(phi), 0, np.cos(phi)],
        ])
        vec_rot = Ry @ (Rx @ vec)
        Xw, Yw, _Zw = vec_rot
        return Xw, Yw
