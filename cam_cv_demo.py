from telloController import TelloController
import time
import cv2
import numpy as np
from camera_calibration.camera_tranceform import CameraTransform
from pid_controller import DronePIDController
import math

tello = TelloController()
cam = CameraTransform("camera_calibration/camera_params.yaml")

WHITE_THRESHOLD = 250
WHITE_THRESHOLD_CONT = 170
ELLIPSIS_AREA_RATIO_RANGE = (0.6, 1.4)
ELLIPSIS_ASPECT_RATIO_THRESHOLD = 1.35
ELLIPSIS_SIZE_THRESHOLD = 50

TARGET_ERROR_THRESHOLD = 0.1   # 목표 오차 임계값 (cm)
TARGET_ERROR_THRESHOLD_CONT = 0.05  # 컨투어 모드에서의 목표 오차 임계값 (cm)

HOLD_TIME = 0.4

target_point = None

tracking_mode = "ellipse"  # "ellipse" or "contour"

current_frame = None
current_threshold_frame = None

def ellipse_aspect_ratio(ellipse):
    major_axis = max(ellipse[1][0], ellipse[1][1])
    minor_axis = min(ellipse[1][0], ellipse[1][1])
    return major_axis / minor_axis if minor_axis > 0 else 0

def frame_callback(frame):
    global current_frame, current_threshold_frame
    current_frame = frame
    white_threshold = WHITE_THRESHOLD
    if tracking_mode == "contour":
        white_threshold = WHITE_THRESHOLD_CONT
    if frame is None:
        return

    gray = frame[:, :, 0]
    _, binary_frame = cv2.threshold(gray, white_threshold, 255, cv2.THRESH_BINARY)
    current_threshold_frame = binary_frame

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
    global current_out_frame
    current_out_frame = show_frame

def main():
    global tello, cam, target_point, tracking_mode, current_frame
    tello.start(motor_on=False)
    tello.set_video_bitrate(tello.BITRATE_1MBPS)
    tello.set_video_fps(tello.FPS_30)
    tello.set_video_resolution(tello.RESOLUTION_480P)
    tello.setUpVideo(show_video=True, camera_direction=TelloController.CAMERA_DOWNWARD, frame_callback=frame_callback)
    tello.printInfo()
    while not tello.can_read_frame():
        time.sleep(0.1)
    print("드론 연결 성공")

    
    tracking_mode = "ellipse"
    input("타원 추적 모드저장.")
    cv2.imwrite("ellipse_tracking.png", current_frame)
    cv2.imwrite("ellipse_tracking_th.png", current_threshold_frame)
    cv2.imwrite("ellipse_tracking_out.png", current_out_frame)

    tracking_mode = "contour"
    input("컨투어 추적 모드저장.")
    cv2.imwrite("contour_tracking.png", current_frame)
    cv2.imwrite("contour_tracking_th.png", current_threshold_frame)
    cv2.imwrite("contour_tracking_out.png", current_out_frame)

    while True:
        time.sleep(0.1)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("프로그램 종료")
    except Exception as e:
        print(f"오류 발생: {e}")
        tello.land()
    finally:
        try:
            tello.land()
        except:
            pass
        tello.__del__()
        print("드론 연결 해제")