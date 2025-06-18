from telloController import TelloController
import time
import cv2
import numpy as np
from camera_calibration.camera_tranceform import CameraTransform
from pid_controller import DronePIDController

tello = TelloController()
cam = CameraTransform("camera_calibration/camera_params.yaml")

WHITE_THRESHOLD = 230
ELLIPSIS_AREA_RATIO_RANGE = (0.8, 1.2)
ELLIPSIS_ASPECT_RATIO_THRESHOLD = 1.2
ELLIPSIS_SIZE_THRESHOLD = 50

target_point = None

def ellipse_aspect_ratio(ellipse):
    major_axis = max(ellipse[1][0], ellipse[1][1])
    minor_axis = min(ellipse[1][0], ellipse[1][1])
    return major_axis / minor_axis if minor_axis > 0 else 0

def frame_callback(frame):
    if frame is None:
        return

    gray = frame[:, :, 0]
    _, binary_frame = cv2.threshold(gray, WHITE_THRESHOLD, 255, cv2.THRESH_BINARY)
    
    show_frame = np.copy(frame)

    # 타원 검출 및 타원에 가까운 것만 남기기
    contours, _ = cv2.findContours(binary_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    eclipses = []
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
    
    if target is not None:
        target_point = (int(target[0][0]), int(target[0][1]))
        cv2.circle(show_frame, target_point, 5, (0, 0, 255), -1)

    for ellipse in eclipses:
        cv2.ellipse(show_frame, ellipse, (0, 255, 0), 2)

    cv2.imshow("show_frame", show_frame)

def main():
    tello.start(motor_on=False)
    tello.set_video_bitrate(tello.BITRATE_2MBPS)
    tello.set_video_fps(tello.FPS_30)
    tello.set_video_resolution(tello.RESOLUTION_480P)
    tello.setUpVideo(show_video=True, camera_direction=TelloController.CAMERA_DOWNWARD, frame_callback=frame_callback)
    tello.printInfo()
    while not tello.can_read_frame():
        time.sleep(0.1)
    print("드론 연결 성공")
    tello.set_speed(100)
    if not tello.can_flight():
        return
    
    # while True:
    #     time.sleep(0.1)

    tello.takeoff()
    time.sleep(1.2)
    height = tello.get_height()
    print(f"현재 높이: {height}cm")
    tello.go_xyz_speed(0, 0, 100 - height, 100)
    pid = DronePIDController(tello)

    while True:
        height = tello.get_height()
        print(f"현재 높이: {height}cm")
        if target_point is not None:
            u, v = target_point
            attitude = tello.get_attitude()
            pitch = attitude['pitch']
            roll = attitude['roll']
            X, Y = cam.uv_to_cm(u, v, height, undistort=True, camera_pitch_deg=pitch, camera_roll_deg=roll)
            print(f"실제 좌표: X={X:.2f}cm, Y={Y:.2f}cm (pitch={pitch:.2f}°, roll={roll:.2f}°)")
            X, Y = int(round(X)), int(round(Y))
            dt = pid.compute_dt()
            if dt == 0:
                time.sleep(0.01)
                continue
            pid.control_position(-X, -Y, dt=dt)
        else:
            print("타겟을 찾을 수 없습니다.")
            time.sleep(0.1)

    tello.takeoff()
    time.sleep(1.2)
    height = tello.get_height()
    print(f"현재 높이: {height}cm")
    tello.go_xyz_speed(80, 20, 150 - height, 100)
    time.sleep(0.5)
    height = tello.get_height()
    print(f"현재 높이: {height}cm")
    tello.go_xyz_speed(160, -10, 100 - height, 100)
    tello.rotate_clockwise(90)
    time.sleep(0.5)
    print(f"현재 높이: {height}cm")
    tello.go_xyz_speed(100, 0, 150 - height, 100)

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