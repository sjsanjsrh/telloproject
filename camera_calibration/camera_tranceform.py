import yaml
import numpy as np
import cv2

class CameraTransform:
    def __init__(self, yaml_path):
        with open(yaml_path, 'r') as f:
            params = yaml.safe_load(f)
        self.K = np.array(params['camera_matrix']['data']).reshape(3, 3)
        self.dist = np.array(params['distortion_coefficients']['data'])
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
        # pitch, roll 모두 적용 (카메라 기준, 오른손 좌표계)
        theta = np.deg2rad(camera_pitch_deg)
        phi = np.deg2rad(camera_roll_deg)
        # 회전 행렬 적용 (pitch: x축, roll: y축)
        # [Xc, Yc, Zc] = R * [x*Z, y*Z, Z]
        Z = float(Z_cm)
        Xc = x * Z
        Yc = y * Z
        vec = np.array([Xc, Yc, Z])
        # pitch(x), roll(y) 회전
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(theta), -np.sin(theta)],
            [0, np.sin(theta), np.cos(theta)]
        ])
        Ry = np.array([
            [np.cos(phi), 0, np.sin(phi)],
            [0, 1, 0],
            [-np.sin(phi), 0, np.cos(phi)]
        ])
        vec_rot = Ry @ (Rx @ vec)
        Xw, Yw, Zw = vec_rot
        return Xw, Yw  # [cm]
