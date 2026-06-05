import argparse
import glob
import os

import cv2
import numpy as np
import yaml


DEFAULT_CHECKERBOARD = (9, 6)
CAMERA_ALIASES = {
    "all": "all",
    "front": "forward",
    "forward": "forward",
    "f": "forward",
    "down": "downward",
    "bottom": "downward",
    "downward": "downward",
    "d": "downward",
}


def parse_args():
    parser = argparse.ArgumentParser(description="Calibrate a Tello camera profile.")
    parser.add_argument(
        "--camera",
        default="downward",
        choices=sorted(CAMERA_ALIASES),
        help="Camera to calibrate: forward/front, downward/bottom, or all.",
    )
    parser.add_argument(
        "--resolution",
        type=int,
        default=480,
        choices=(480, 720),
        help="Video resolution profile to calibrate.",
    )
    parser.add_argument(
        "--images",
        default=None,
        help="Glob for calibration images. Defaults to calib_images/<camera>_<resolution>p/*.png, then calib_images/*.png.",
    )
    parser.add_argument(
        "--output",
        default=os.path.join(os.path.dirname(__file__), "camera_params.yaml"),
        help="YAML file to update.",
    )
    parser.add_argument(
        "--checkerboard",
        default=f"{DEFAULT_CHECKERBOARD[0]}x{DEFAULT_CHECKERBOARD[1]}",
        help="Checkerboard inner corner count, e.g. 9x6.",
    )
    return parser.parse_args()


def normalize_camera(camera):
    return CAMERA_ALIASES[str(camera).lower()]


def profile_name(camera, resolution):
    if normalize_camera(camera) == "downward":
        return "downward"
    return f"{normalize_camera(camera)}_{int(resolution)}p"


def parse_checkerboard(value):
    try:
        width, height = value.lower().split("x", 1)
        return int(width), int(height)
    except ValueError as exc:
        raise ValueError("--checkerboard must look like 9x6") from exc


def default_image_glob(camera, resolution):
    camera = normalize_camera(camera)
    base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    dirname = "downward" if camera == "downward" else f"{camera}_{resolution}p"
    profile_glob = os.path.join(base_dir, "calib_images", dirname, "*.png")
    if glob.glob(profile_glob):
        return profile_glob
    return os.path.join(base_dir, "calib_images", "*.png")


def build_object_points(checkerboard):
    objp = np.zeros((checkerboard[0] * checkerboard[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:checkerboard[0], 0:checkerboard[1]].T.reshape(-1, 2)
    return objp


def load_existing_yaml(path):
    if not os.path.exists(path):
        return {}
    with open(path, "r", encoding="utf-8") as file_handle:
        return yaml.safe_load(file_handle) or {}


def migrate_legacy_params(params):
    if "profiles" in params:
        return params
    if "camera_matrix" not in params:
        return params
    return {
        "default_profile": "legacy",
        "profiles": {
            "legacy": params,
        },
    }


class NoAliasDumper(yaml.SafeDumper):
    def ignore_aliases(self, data):
        return True


def calibration_jobs(camera, resolution):
    if camera == "all":
        return [
            ("downward", 480),
            ("forward", 480),
            ("forward", 720),
        ]
    return [(camera, resolution)]


def calibrate_profile(camera, resolution, checkerboard, image_glob=None):
    image_glob = image_glob or default_image_glob(camera, resolution)
    images = sorted(glob.glob(image_glob))

    print(f"\nProfile: {profile_name(camera, resolution)}")
    print(f"Image glob: {image_glob}")
    print(f"Image count: {len(images)}")

    objp = build_object_points(checkerboard)
    obj_points = []
    img_points = []
    gray = None

    for fname in images:
        img = cv2.imread(fname)
        if img is None:
            print(f"Skipped unreadable image: {fname}")
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, checkerboard, None)
        if found:
            obj_points.append(objp)
            corners2 = cv2.cornerSubPix(
                gray,
                corners,
                (11, 11),
                (-1, -1),
                criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
            )
            img_points.append(corners2)
        else:
            print(f"Checkerboard corners not found: {fname}")

    if not obj_points or gray is None:
        raise RuntimeError(f"No valid checkerboard corners were detected for {profile_name(camera, resolution)}.")

    ret, mtx, dist, _rvecs, _tvecs = cv2.calibrateCamera(obj_points, img_points, gray.shape[::-1], None, None)
    if not ret:
        raise RuntimeError("Calibration failed.")

    print("\nCalibration succeeded.")
    print("\nCamera matrix:")
    print(mtx)
    print("\nDistortion coefficients:")
    print(dist)

    calib_result = {
        "camera": camera,
        "image_width": int(gray.shape[1]),
        "image_height": int(gray.shape[0]),
        "camera_matrix": {
            "rows": 3,
            "cols": 3,
            "data": mtx.flatten().tolist(),
        },
        "distortion_coefficients": {
            "rows": 1,
            "cols": int(dist.size),
            "data": dist.flatten().tolist(),
        },
    }
    if camera != "downward":
        calib_result["resolution"] = int(resolution)

    return profile_name(camera, resolution), calib_result


def main():
    args = parse_args()
    camera = normalize_camera(args.camera)
    resolution = args.resolution
    checkerboard = parse_checkerboard(args.checkerboard)

    if camera == "all" and args.images is not None:
        raise ValueError("--images cannot be used with --camera all. Use per-profile default folders instead.")

    save_path = os.path.abspath(args.output)
    os.makedirs(os.path.dirname(save_path), exist_ok=True)
    params = migrate_legacy_params(load_existing_yaml(save_path))
    params.setdefault("profiles", {})

    selected_profile = None
    for job_camera, job_resolution in calibration_jobs(camera, resolution):
        selected_profile, calib_result = calibrate_profile(
            job_camera,
            job_resolution,
            checkerboard,
            image_glob=args.images,
        )
        params["profiles"][selected_profile] = calib_result

    if camera == "all" and "downward" in params["profiles"]:
        selected_profile = "downward"
        calib_result = params["profiles"][selected_profile]

    params["default_profile"] = selected_profile

    # Keep top-level keys for older scripts that still read a single profile.
    params.update(calib_result)

    with open(save_path, "w", encoding="utf-8") as file_handle:
        yaml.dump(params, file_handle, Dumper=NoAliasDumper, sort_keys=False)

    print(f"\nSaved calibration profile(s) to: {save_path}")


if __name__ == "__main__":
    main()
