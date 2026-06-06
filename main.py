from telloController import TelloController
import argparse
import time
import cv2
import numpy as np
from camera_calibration.camera_tranceform import CameraTransform
from pid_controller import DronePIDController
import math
from pathlib import Path

import yaml

tello = TelloController()
CAMERA_DIRECTION = TelloController.CAMERA_DOWNWARD
cam = CameraTransform(
    "camera_calibration/camera_params.yaml",
    camera_direction="downward",
)

WHITE_THRESHOLD = 250
WHITE_THRESHOLD_CONT = 150
ELLIPSIS_AREA_RATIO_RANGE = (0.6, 1.4)
ELLIPSIS_ASPECT_RATIO_THRESHOLD = 1.35
ELLIPSIS_SIZE_THRESHOLD = 50

TARGET_ERROR_THRESHOLD = 0.15   # 목표 오차 임계값 (cm)
TARGET_ERROR_THRESHOLD_CONT = 0.025  # 컨투어 모드에서의 목표 오차 임계값 (cm)

COUNTOUR_HEIGHT = 50

HOLD_TIME = 0.6
DEFAULT_FLIGHT_PLAN = Path("flight_path.yaml")

target_point = None

tracking_mode = "init"  # "ellipse" or "contour"

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


def parse_args():
    parser = argparse.ArgumentParser(description="Tello waypoint flight and landing controller")
    parser.add_argument("--flight-plan", default=str(DEFAULT_FLIGHT_PLAN), help="Flight plan YAML path")
    return parser.parse_args()


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


def execute_flight_step(tello, step, index=None):
    name = str(step.get("name", f"step {index}" if index is not None else "step"))

    height = tello.get_height()
    print(f"현재 높이: {height}cm")

    if step.get("move_cm") is not None:
        x, y, z = vector3(step["move_cm"], f"{name}.move_cm")
        speed = int(step.get("speed", 70))
        tello.go_xyz_speed(x, y, z, speed)
        print(f"{name} 이동 완료: x={x}, y={y}, z={z}, speed={speed}")
    elif step.get("rotate_deg") is not None:
        rotate_deg = int(step["rotate_deg"])
        if rotate_deg >= 0:
            tello.rotate_clockwise(rotate_deg)
        else:
            tello.rotate_counter_clockwise(abs(rotate_deg))
        print(f"{name} 회전 완료: {rotate_deg}도")
    elif step.get("wait_sec") is not None:
        print(f"{name} 대기")
    else:
        raise ValueError(f"{name} must define move_cm, rotate_deg, or wait_sec")

    wait_sec = float(step.get("wait_sec", 0.0))
    if wait_sec > 0.0:
        time.sleep(wait_sec)


def execute_flight_plan(tello, plan):
    start_step = plan.get("start")
    if start_step:
        if start_step.get("takeoff_position_cm") is not None:
            print(f"{start_step.get('takeoff_name', 'takeoff')} 상대 위치: {start_step['takeoff_position_cm']}")
        elif start_step.get("takeoff_height_cm") is not None:
            print(f"{start_step.get('takeoff_name', 'takeoff')} 기준 높이: {float(start_step['takeoff_height_cm']):.1f}cm")
        execute_flight_step(tello, start_step, index=0)

    for index, waypoint in enumerate(plan.get("waypoints", []), start=1):
        execute_flight_step(tello, waypoint, index=index)

def ellipse_aspect_ratio(ellipse):
    """타원의 장축과 단축 비율을 계산합니다."""
    major_axis = max(ellipse[1][0], ellipse[1][1])
    minor_axis = min(ellipse[1][0], ellipse[1][1])
    return major_axis / minor_axis if minor_axis > 0 else 0

def frame_callback(frame):
    """프레임 콜백 함수, 드론의 카메라 영상 처리 및 타겟 추적을 수행합니다."""
    target = None

    white_threshold = WHITE_THRESHOLD
    if tracking_mode == "contour":
        white_threshold = WHITE_THRESHOLD_CONT
    if frame is None:
        return

    gray = frame[:, :, 0]
    _, binary_frame = cv2.threshold(gray, white_threshold, 255, cv2.THRESH_BINARY)
    
    show_frame = np.copy(frame)

    contours, _ = cv2.findContours(binary_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    eclipses = []

    if tracking_mode == "ellipse":
        # 타원 검출 및 타원에 가까운 것만 남기기
        for cnt in contours:
            if len(cnt) >= 5:
                ellipse = cv2.fitEllipse(cnt)
                contour_area = cv2.contourArea(cnt)
                ellipse_area = np.pi * (ellipse[1][0]/2) * (ellipse[1][1]/2)
                area_ratio = contour_area / ellipse_area if ellipse_area > 0 else 0
                # 타원의 면적 비율과 장축/단축 비율을 기준으로 필터링
                # 타원의 면적 비율이 ELLIPSIS_AREA_RATIO_RANGE 범위 내에 있고,
                # 타원의 장축/단축 비율이 ELLIPSIS_ASPECT_RATIO_THRESHOLD 이하이며,
                # 컨투어 면적이 ELLIPSIS_SIZE_THRESHOLD 이상인 경우에만 타원으로 간주
                if ELLIPSIS_AREA_RATIO_RANGE[0] < area_ratio < ELLIPSIS_AREA_RATIO_RANGE[1] and \
                        ellipse_aspect_ratio(ellipse) < ELLIPSIS_ASPECT_RATIO_THRESHOLD and \
                        contour_area > ELLIPSIS_SIZE_THRESHOLD:
                    eclipses.append(ellipse)

        global target_point
        target = None
        target_point = None
        if eclipses:
            target = max(eclipses, key=lambda e: e[1][0] * e[1][1])  # 가장 큰 타원을 목표로 설정
    elif tracking_mode == "contour":
        target = max(contours, key=cv2.contourArea, default=None)


    if target is not None:
        if tracking_mode == "ellipse":
            target_point = (int(target[0][0]), int(target[0][1]))
        elif tracking_mode == "contour":
            M = cv2.moments(target)
            if M["m00"] != 0:
                target_point = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        if target_point is not None:
            cv2.circle(show_frame, target_point, 5, (0, 0, 255), -1)

    # 타원과 컨투어를 그리기
    for ellipse in eclipses:
        cv2.ellipse(show_frame, ellipse, (0, 255, 0), 2)
    cv2.drawContours(show_frame, contours, -1, (255, 0, 0), 1)

    cv2.putText(show_frame, f"{tracking_mode}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.imshow("show_frame", show_frame)

def main():
    global target_point, tracking_mode
    args = parse_args()
    flight_plan = load_flight_plan(args.flight_plan)

    tello.start(motor_on=False)
    tello.set_video_bitrate(tello.BITRATE_1MBPS)
    tello.set_video_fps(tello.FPS_30)
    tello.set_video_resolution(tello.RESOLUTION_480P)
    tello.setUpVideo(show_video=True, camera_direction=CAMERA_DIRECTION, frame_callback=frame_callback)
    tello.printInfo()
    while not tello.can_read_frame():
        time.sleep(0.1)
    print("드론 연결 성공")
    tello.set_speed(int(flight_plan.get("speed", 50)))
    if not tello.can_flight():
        return
    
    start_time = time.time()
    
    tello.takeoff()
    print("이륙 완료")

    execute_flight_plan(tello, flight_plan)

    fly_time = time.time()
    print(f" 비행 시간: {fly_time - start_time:.2f}초")

    # 착륙
    print("착륙진행")
    height = tello.get_height()
    print(f"현재 높이: {height}cm")
    holdtime = HOLD_TIME
    tracking_mode = "ellipse"
    current_holdtime = holdtime
    pid = DronePIDController(tello)
    pid.init_dt()

    while tracking_mode != "land":
        height = tello.get_height()
        height = max(height, 20)
        if height <= COUNTOUR_HEIGHT:
            tracking_mode = "contour"
        print(f"현재 높이: {height}cm")
        if target_point is not None:
            u, v = target_point
            X, Y = cam.uv_to_cm(u, v, height, undistort=True)
            dt = pid.compute_dt()
            if dt == 0:
                time.sleep(0.01)
                continue
            error = math.sqrt(X**2 + Y**2) / height
            if error > (TARGET_ERROR_THRESHOLD if tracking_mode != "contour" else TARGET_ERROR_THRESHOLD_CONT):
                # 목표 위치가 너무 멀면 PID 제어
                current_holdtime = holdtime
                print(f"목표 위치: ({X:.2f}, {Y:.2f}) cm, 오차: {error:.2f}")
                pid.control_position(X, Y, dt=dt)
            else:
                # 목표 위치에 도달하면 holdtime 카운트다운
                current_holdtime -= dt
                print(f"목표 위치 도달, holdtime: {current_holdtime:.2f}초")
            if current_holdtime <= 0:
                # holdtime이 0 이하가 되면 드론 하강
                print("목표 위치 도달, 하강 시작")
                if tracking_mode != "contour":
                    tello.go_xyz_speed(0, 0, -50, 50)
                    pid.reset()
                    pid.init_dt()
                else:
                    tello.land()
                    pid.reset()
                    pid.init_dt()
                    tracking_mode = "land"
        else:
            time.sleep(0.001)
            continue
        
        time.sleep(0.1)

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
        try:
            tello.land()
        except:
            pass
        tello.__del__()
        print("드론 연결 해제")
