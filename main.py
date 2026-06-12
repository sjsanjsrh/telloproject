from telloController import TelloController
from threading import Thread
import time
import numpy as np
from pathlib import Path
from types import SimpleNamespace

import yaml
from landing_controller import LandingConfig, LandingController
from yolo_seg.pose_worker import TelloPoseWorker
from yolo_seg.waypoint_correction import WaypointCorrector
from yolo_seg.world_pose import yaw_rotation_matrix_z

tello = None
tello_started = False
landing_controller = None
pose_worker = None
FLIGHT_PLAN_PATH = "flight_path.yaml"

POSE_WORKER_ENABLED = True
POSE_MODEL_PATH = "yolo_seg/res/runs/segment/train-26/weights/best.pt"
CAMERA_PARAMS_PATH = "camera_calibration/camera_params.yaml"
CAMERA_PROFILE = "forward_480p"
CAMERA_NAME = "forward"
CAMERA_RESOLUTION = 480
VIDEO_FPS = 30.0
SCENE_MAP_PATH = "yolo_seg/obstacles.yaml"

MIN_AREA_PX = 200.0

# YOLO run args are loaded from the model run directory first.
YOLO_DEVICE = "auto"
YOLO_FPS = 8.0
POSE_PANEL_HEIGHT = 360
POSE_WINDOW_NAME = "main_pose_worker"
DISABLE_POSE_VIZ = False
DISABLE_3D = False
SCENE_3D_RENDERER = "gpu"

COMMAND_PRIOR_ENABLED = True
COMMAND_SPEED_CM_S = 18.0 # Tello의 최대 속도는 약 48cm/s이지만 왜 인지 18cm/s로 보임
COMMAND_NOISE_CM = 700.0
COMMAND_START_INDEX = 0
DISABLE_PNP_FUSION = False

SLAM_BACKEND = "orbslam3_py"
SLAM_FPS = 20.0
SLAM_SCALE_CM = 200.0
ORBSLAM3_VOCAB = "third_party/orbslam3_windows/ORB_SLAM3/Vocabulary/ORBvoc.txt"
ORBSLAM3_SETTINGS = "yolo_seg/orbslam3_tello_forward_480p.yaml"
ORBSLAM3_PY_MODULE = "third_party/orbslam3_py/orbslam3_py.cp310-win_amd64.pyd"

WAYPOINT_CORRECTION_ENABLED = False
APPLY_WAYPOINT_CORRECTION = False
CORRECTION_GAIN = 0.4
MAX_CORRECTION_CM = 25.0
MIN_CORRECTION_CONFIDENCE = 0.45
POSE_MAX_AGE_SEC = 1.0
POSE_WORKER_STARTUP_TIMEOUT_SEC = 120.0
PRE_LANDING_VIDEO_STEP_NAME = "w4"

DEFAULT_PLAN = {
    "speed": 50,
    "start": {
        "name": "hover",
        "position_cm": [0, 0, 0],
        "takeoff_name": "takeoff",
        "takeoff_position_cm": [0, 0, 80],
        "move_cm": [0, 0, 40],
        "speed": 70,
        "wait_sec": 1.2,
    },
    "waypoints": [
        {"name": "waypoint 1", "move_cm": [80, -15, 30], "speed": 90, "wait_sec": 0.5},
        {"name": "waypoint 2", "move_cm": [150, 0, -70], "speed": 90},
        {"name": "rotate 90", "rotate_deg": 90, "wait_sec": 0.5},
        {"name": "waypoint 3", "move_cm": [80, -28, 70], "speed": 90},
        {"name": "waypoint 4", "move_cm": [142, 0, 0], "speed": 90},
    ],
}


def load_yolo_run_args(model_path: str) -> dict:
    model = Path(model_path)
    candidates = []
    for parent in [model.parent, *model.parents]:
        candidates.append(parent / "args.yaml")
        if parent.name == "weights":
            candidates.append(parent.parent / "args.yaml")

    for candidate in candidates:
        if not candidate.exists():
            continue
        with open(candidate, "r", encoding="utf-8") as file_handle:
            data = yaml.safe_load(file_handle) or {}
        print(f"YOLO run args: {candidate}")
        return data
    searched = ", ".join(str(path) for path in candidates)
    raise FileNotFoundError(f"YOLO run args.yaml not found for model: {model_path}. Searched: {searched}")


def require_yolo_run_value(run_args: dict, key: str):
    if key not in run_args or run_args[key] is None:
        raise KeyError(f"YOLO run args.yaml missing required key: {key}")
    return run_args[key]


def yolo_run_int(run_args: dict, key: str) -> int:
    value = require_yolo_run_value(run_args, key)
    if isinstance(value, (list, tuple)):
        if not value:
            raise ValueError(f"YOLO run args.yaml key is empty: {key}")
        value = value[0]
    return int(value)


def make_pose_settings():
    yolo_run_args = load_yolo_run_args(POSE_MODEL_PATH)
    yolo_imgsz = yolo_run_int(yolo_run_args, "imgsz")
    yolo_conf = float(require_yolo_run_value(yolo_run_args, "conf"))
    return SimpleNamespace(
        flight_plan=FLIGHT_PLAN_PATH,
        pose_worker=POSE_WORKER_ENABLED,
        pose_model=POSE_MODEL_PATH,
        camera_params=CAMERA_PARAMS_PATH,
        camera_profile=CAMERA_PROFILE,
        camera=CAMERA_NAME,
        resolution=CAMERA_RESOLUTION,
        camera_direction=CAMERA_NAME,
        video_resolution=CAMERA_RESOLUTION,
        effective_camera_profile=CAMERA_PROFILE,
        effective_camera_direction=CAMERA_NAME,
        effective_resolution=CAMERA_RESOLUTION,
        video_fps=VIDEO_FPS,
        scene_map=SCENE_MAP_PATH,
        min_area_px=MIN_AREA_PX,
        imgsz=yolo_imgsz,
        conf=yolo_conf,
        device=YOLO_DEVICE,
        yolo_fps=YOLO_FPS,
        panel_height=POSE_PANEL_HEIGHT,
        pose_window_name=POSE_WINDOW_NAME,
        no_pose_viz=DISABLE_POSE_VIZ,
        no_3d=DISABLE_3D,
        scene_3d_renderer=SCENE_3D_RENDERER,
        command_prior=COMMAND_PRIOR_ENABLED,
        command_speed_cm_s=COMMAND_SPEED_CM_S,
        command_noise_cm=COMMAND_NOISE_CM,
        command_start_index=COMMAND_START_INDEX,
        disable_pnp_fusion=DISABLE_PNP_FUSION,
        slam_backend=SLAM_BACKEND,
        slam_fps=SLAM_FPS,
        slam_scale_cm=SLAM_SCALE_CM,
        slam_input="bgr",
        orbslam3_vocab=ORBSLAM3_VOCAB,
        orbslam3_settings=ORBSLAM3_SETTINGS,
        orbslam3_py_module=ORBSLAM3_PY_MODULE,
        enable_waypoint_correction=WAYPOINT_CORRECTION_ENABLED,
        apply_waypoint_correction=APPLY_WAYPOINT_CORRECTION,
        correction_gain=CORRECTION_GAIN,
        max_correction_cm=MAX_CORRECTION_CM,
        min_correction_confidence=MIN_CORRECTION_CONFIDENCE,
        pose_max_age_sec=POSE_MAX_AGE_SEC,
    )


def wait_for_frame_stream(tello, label: str = "camera", interval_sec: float = 0.2) -> None:
    while not tello.can_read_frame():
        print(f"{label}: waiting for frame...")
        time.sleep(interval_sec)


def preload_landing_video(tello, landing_controller, active_pose_worker=None):
    global pose_worker
    if active_pose_worker is not None:
        Thread(target=active_pose_worker.stop, daemon=True).start()
    pose_worker = None
    print("preloading landing video")
    tello.switchVideoProcessing(
        show_video=False,
        camera_direction=TelloController.CAMERA_DOWNWARD,
        frame_callback=landing_controller.frame_callback,
    )
    return None


def ensure_landing_video(tello, landing_controller, timeout_sec: float = 4.0):
    print("ensuring landing video")
    tello.switchVideoProcessing(
        show_video=False,
        camera_direction=TelloController.CAMERA_DOWNWARD,
        frame_callback=landing_controller.frame_callback,
    )
    deadline = time.time() + float(timeout_sec)
    while time.time() < deadline:
        if tello.can_read_frame() and tello.get_frame() is not None:
            return
        print("landing video: waiting for frame...")
        time.sleep(0.2)


def load_flight_plan(path):
    plan_path = Path(path)
    if not plan_path.exists():
        print(f"비행 경로 파일 없음, 기본 경로 사용: {plan_path}")
        return DEFAULT_PLAN

    with open(plan_path, "r", encoding="utf-8") as file_handle:
        data = yaml.safe_load(file_handle) or {}
    return data


def vector3(value, field_name):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"{field_name} must be [x, y, z]")
    return int(value[0]), int(value[1]), int(value[2])


def make_flight_state(plan):
    start = plan.get("start", {}) or {}
    return {
        "position_cm": np.asarray(start.get("position_cm", [0.0, 0.0, 0.0]), dtype=np.float64).reshape(3),
        "yaw_deg": float(start.get("yaw_deg", 0.0)),
    }


def update_flight_state(flight_state, step):
    if flight_state is None:
        return
    if step.get("move_cm") is not None:
        move = np.asarray(vector3(step["move_cm"], "move_cm"), dtype=np.float64)
        flight_state["position_cm"] = flight_state["position_cm"] + yaw_rotation_matrix_z(flight_state["yaw_deg"]) @ move
    if step.get("rotate_deg") is not None:
        flight_state["yaw_deg"] = float(flight_state["yaw_deg"]) + float(step["rotate_deg"])


def apply_takeoff_state(flight_state, plan):
    if flight_state is None:
        return
    start = plan.get("start", {}) or {}
    if start.get("takeoff_position_cm") is not None:
        flight_state["position_cm"] = flight_state["position_cm"] + np.asarray(
            vector3(start["takeoff_position_cm"], "start.takeoff_position_cm"),
            dtype=np.float64,
        )
    elif start.get("takeoff_height_cm") is not None:
        flight_state["position_cm"] = flight_state["position_cm"] + np.array(
            [0.0, 0.0, float(start["takeoff_height_cm"])],
            dtype=np.float64,
        )


def publish_command_pose(pose_worker, flight_state, status: str, segment_index: int = 0):
    if pose_worker is None or flight_state is None:
        return
    pose_worker.set_command_pose(
        position_cm=flight_state["position_cm"],
        rotation_matrix=yaw_rotation_matrix_z(flight_state["yaw_deg"]),
        status=status,
        segment_index=segment_index,
    )


def command_state_after_move(flight_state, move_cm):
    if flight_state is None:
        return None
    move = np.asarray(move_cm, dtype=np.float64).reshape(3)
    return {
        "position_cm": flight_state["position_cm"] + yaw_rotation_matrix_z(flight_state["yaw_deg"]) @ move,
        "yaw_deg": float(flight_state["yaw_deg"]),
    }


def command_state_after_rotate(flight_state, rotate_deg):
    if flight_state is None:
        return None
    return {
        "position_cm": np.asarray(flight_state["position_cm"], dtype=np.float64).reshape(3).copy(),
        "yaw_deg": float(flight_state["yaw_deg"]) + float(rotate_deg),
    }


def apply_flight_state(flight_state, next_state):
    if flight_state is None or next_state is None:
        return
    flight_state["position_cm"] = np.asarray(next_state["position_cm"], dtype=np.float64).reshape(3)
    flight_state["yaw_deg"] = float(next_state["yaw_deg"])


def execute_flight_step(tello, step, index=None, pose_worker=None, waypoint_corrector=None, flight_state=None, pose_max_age_sec=1.0):
    name = str(step.get("name", f"step {index}" if index is not None else "step"))

    height = tello.get_height()
    print(f"현재 높이: {height}cm")

    if step.get("move_cm") is not None:
        x, y, z = vector3(step["move_cm"], f"{name}.move_cm")
        speed = int(step.get("speed", 70))
        if waypoint_corrector is not None and flight_state is not None:
            pose_result = pose_worker.latest(max_age_sec=pose_max_age_sec) if pose_worker is not None else None
            best_pose = pose_result.best_pose if pose_result is not None else None
            confidence = pose_result.pose_confidence if pose_result is not None else 0.0
            correction = waypoint_corrector.correct_move(
                planned_move_cm=(x, y, z),
                commanded_position_cm=flight_state["position_cm"],
                estimated_position_cm=best_pose.position_cm if best_pose is not None else None,
                yaw_deg=flight_state["yaw_deg"],
                confidence=confidence,
            )
            x, y, z = correction.corrected_move_cm
            print(
                f"{name} correction {correction.reason}: "
                f"err={tuple(round(v, 1) for v in correction.error_world_cm)} "
                f"corr={tuple(round(v, 1) for v in correction.correction_local_cm)} "
                f"move={correction.original_move_cm}->{correction.corrected_move_cm}"
            )
        next_state = command_state_after_move(flight_state, (x, y, z))
        publish_command_pose(pose_worker, next_state, f"{name} cmd", segment_index=index or 0)
        tello.go_xyz_speed(x, y, z, speed)
        apply_flight_state(flight_state, next_state)
        publish_command_pose(pose_worker, flight_state, f"{name} ack", segment_index=index or 0)
        print(f"{name} 이동 완료: x={x}, y={y}, z={z}, speed={speed}")
    elif step.get("rotate_deg") is not None:
        rotate_deg = int(step["rotate_deg"])
        next_state = command_state_after_rotate(flight_state, rotate_deg)
        publish_command_pose(pose_worker, next_state, f"{name} cmd", segment_index=index or 0)
        if rotate_deg >= 0:
            tello.rotate_clockwise(rotate_deg)
        else:
            tello.rotate_counter_clockwise(abs(rotate_deg))
        apply_flight_state(flight_state, next_state)
        publish_command_pose(pose_worker, flight_state, f"{name} ack", segment_index=index or 0)
        print(f"{name} 회전 완료: {rotate_deg}도")
    elif step.get("wait_sec") is not None:
        print(f"{name} 대기")
    else:
        raise ValueError(f"{name} must define move_cm, rotate_deg, or wait_sec")

    wait_sec = float(step.get("wait_sec", 0.0))
    if wait_sec > 0.0:
        time.sleep(wait_sec)


def execute_flight_plan(
    tello,
    plan,
    pose_worker=None,
    waypoint_corrector=None,
    pose_max_age_sec=1.0,
    takeoff_acknowledged=False,
    landing_controller=None,
    pre_landing_step_name=None,
):
    flight_state = make_flight_state(plan)
    publish_command_pose(pose_worker, flight_state, "start", segment_index=0)
    if takeoff_acknowledged:
        apply_takeoff_state(flight_state, plan)
        publish_command_pose(pose_worker, flight_state, "takeoff ack", segment_index=0)
    start_step = plan.get("start")
    if start_step:
        if start_step.get("takeoff_position_cm") is not None:
            print(f"{start_step.get('takeoff_name', 'takeoff')} 상대 위치: {start_step['takeoff_position_cm']}")
        elif start_step.get("takeoff_height_cm") is not None:
            print(f"{start_step.get('takeoff_name', 'takeoff')} 기준 높이: {float(start_step['takeoff_height_cm']):.1f}cm")
        execute_flight_step(
            tello,
            start_step,
            index=0,
            pose_worker=pose_worker,
            waypoint_corrector=waypoint_corrector,
            flight_state=flight_state,
            pose_max_age_sec=pose_max_age_sec,
        )

    for index, waypoint in enumerate(plan.get("waypoints", []), start=1):
        if (
            pre_landing_step_name is not None
            and landing_controller is not None
            and str(waypoint.get("name", f"waypoint {index}")) == str(pre_landing_step_name)
        ):
            pose_worker = preload_landing_video(tello, landing_controller, pose_worker)
            pre_landing_step_name = None
        execute_flight_step(
            tello,
            waypoint,
            index=index,
            pose_worker=pose_worker,
            waypoint_corrector=waypoint_corrector,
            flight_state=flight_state,
            pose_max_age_sec=pose_max_age_sec,
        )

def main():
    global tello_started, tello, landing_controller, pose_worker
    settings = make_pose_settings()
    flight_plan = load_flight_plan(FLIGHT_PLAN_PATH)
    waypoint_corrector = None
    tello = TelloController()
    landing_controller = LandingController(tello, LandingConfig())

    tello.start(motor_on=False)
    tello_started = True
    tello.set_video_bitrate(tello.BITRATE_1MBPS)
    tello.set_video_fps(tello.FPS_30)
    tello.set_video_resolution(tello.RESOLUTION_480P if CAMERA_RESOLUTION == 480 else tello.RESOLUTION_720P)
    if POSE_WORKER_ENABLED:
        camera_direction = TelloController.CAMERA_FORWARD if CAMERA_NAME == "forward" else TelloController.CAMERA_DOWNWARD
        print(f"startup video: direction={camera_direction}, resolution={CAMERA_RESOLUTION}, pose_worker={POSE_WORKER_ENABLED}")
        tello.setUpVideo(show_video=False, camera_direction=camera_direction, frame_callback=None)
    else:
        print(f"startup video: direction={TelloController.CAMERA_DOWNWARD}, resolution={CAMERA_RESOLUTION}, pose_worker={POSE_WORKER_ENABLED}")
        landing_controller.setup_video(show_video=True)
    tello.printInfo()
    wait_for_frame_stream(tello, "startup video")
    print("드론 연결 성공")
    tello.set_speed(int(flight_plan.get("speed", 50)))
    if not tello.can_flight():
        return
    if WAYPOINT_CORRECTION_ENABLED:
        waypoint_corrector = WaypointCorrector(
            gain=CORRECTION_GAIN,
            max_correction_cm=MAX_CORRECTION_CM,
            min_confidence=MIN_CORRECTION_CONFIDENCE,
            dry_run=not APPLY_WAYPOINT_CORRECTION,
        )

    input("이륙 준비 완료. 엔터 키를 눌러 이륙합니다...")
    start_time = time.time()
    tello.takeoff()
    print("이륙 완료")
    
    if POSE_WORKER_ENABLED:
        pose_worker = TelloPoseWorker(tello, settings)
        pose_worker.start()
        print("pose backend loading")

    execute_flight_plan(
        tello,
        flight_plan,
        pose_worker=pose_worker,
        waypoint_corrector=waypoint_corrector,
        pose_max_age_sec=POSE_MAX_AGE_SEC,
        takeoff_acknowledged=True,
        landing_controller=landing_controller,
        pre_landing_step_name=PRE_LANDING_VIDEO_STEP_NAME,
    )

    fly_time = time.time()
    print(f" 비행 시간: {fly_time - start_time:.2f}초")

    ensure_landing_video(tello, landing_controller)
    landing_controller.run()

    land_time = time.time()
    
    print(f"비행 시간: {fly_time - start_time:.2f}초")
    print(f"착륙까지 걸린 시간: {land_time - start_time:.2f}초")


    time.sleep(3.0)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("프로그램 종료")
    except Exception as e:
        print(f"오류 발생: {e}")
    finally:
        if pose_worker is not None:
            pose_worker.stop(timeout=5.0)
        if tello_started and tello is not None:
            try:
                tello.land()
            except:
                pass
            try:
                tello.closseVideo()
            except:
                pass
            try:
                tello.__del__()
            except:
                pass
            print("tello disconnected")
