import time
import os

from threading import Lock

try:
    import pygame
except ImportError as exc:
    raise SystemExit(
        "pygame is required for Xbox controller input. Install it with 'pip install pygame'."
    ) from exc

import cv2

from telloController import TelloController


MAX_RC_SPEED = 100
DEADZONE = 0.12
CONTROL_HZ = 20

LEFT_RIGHT_AXIS = 0
FORWARD_BACK_AXIS = 1
YAW_AXIS = 2
UP_DOWN_AXIS = 3

SAVE_DIR = os.path.join(".", "yolo_seg", "data", "img")
os.makedirs(SAVE_DIR, exist_ok=True)

latest_frame = None
frame_lock = Lock()
capture_index = 0
camera_forward = True


def apply_deadzone(value: float, deadzone: float = DEADZONE) -> float:
    if abs(value) < deadzone:
        return 0.0
    return value


def axis_to_rc(value: float, max_speed: int = MAX_RC_SPEED) -> int:
    value = max(-1.0, min(1.0, value))
    return int(value * max_speed)


def read_sticks(joystick: pygame.joystick.Joystick):
    left_right = axis_to_rc(apply_deadzone(joystick.get_axis(LEFT_RIGHT_AXIS)))
    forward_back = axis_to_rc(-apply_deadzone(joystick.get_axis(FORWARD_BACK_AXIS)))
    up_down = axis_to_rc(-apply_deadzone(joystick.get_axis(UP_DOWN_AXIS)))
    yaw = axis_to_rc(apply_deadzone(joystick.get_axis(YAW_AXIS)))
    return yaw, up_down, forward_back, left_right


def frame_callback(frame):
    global latest_frame
    if frame is None:
        return

    with frame_lock:
        latest_frame = frame.copy()


def save_current_frame():
    global capture_index

    with frame_lock:
        if latest_frame is None:
            print("저장할 프레임이 아직 없습니다.")
            return
        frame = latest_frame.copy()

    filename = os.path.join(SAVE_DIR, f"image_{capture_index:04d}.png")
    cv2.imwrite(filename, frame)
    print(f"저장됨: {filename}")
    capture_index += 1


def main():
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        raise SystemExit("Xbox 패드를 찾지 못했습니다. 패드를 연결한 뒤 다시 실행하세요.")

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"조이스틱 연결됨: {joystick.get_name()}")

    tello = TelloController()
    flying = False
    global camera_forward

    try:
        tello.start(motor_on=False)
        tello.set_video_bitrate(tello.BITRATE_1MBPS)
        tello.set_video_fps(tello.FPS_30)
        tello.set_video_resolution(tello.RESOLUTION_480P)
        tello.setUpVideo(
            show_video=True,
            camera_direction=TelloController.CAMERA_FORWARD,
            frame_callback=frame_callback,
        )

        tello.printInfo()
        while not tello.can_read_frame():
            time.sleep(0.05)

        if not tello.can_flight():
            return

        print("조작 방법:")
        print("A: 이륙")
        print("B: 착륙")
        print("X: 현재 프레임 저장")
        print("Y: 전면/하단 카메라 전환")
        print("Back: 종료")
        print("오른쪽 스틱: 좌우/전후")
        print("왼쪽 스틱: 상승/하강, 요 회전")

        clock = pygame.time.Clock()

        while True:
            pygame.event.pump()

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    raise KeyboardInterrupt
                if event.type == pygame.JOYBUTTONDOWN:
                    if event.button == 0 and not flying:
                        print("이륙")
                        tello.takeoff()
                        flying = True
                    elif event.button == 1 and flying:
                        print("착륙")
                        tello.send_rc_control(0, 0, 0, 0)
                        tello.land()
                        flying = False
                    elif event.button == 2:
                        save_current_frame()
                    elif event.button == 3:
                        camera_forward = not camera_forward
                        if camera_forward:
                            print("전면카메라로 전환")
                            tello.switch_video_direction(TelloController.CAMERA_FORWARD)
                        else:
                            print("하단카메라로 전환")
                            tello.switch_video_direction(TelloController.CAMERA_DOWNWARD)
                    elif event.button == 6:
                        raise KeyboardInterrupt

            if flying:
                left_right, forward_back, up_down, yaw = read_sticks(joystick)
                tello.send_rc_control(left_right, forward_back, up_down, yaw)
            else:
                tello.send_rc_control(0, 0, 0, 0)

            clock.tick(CONTROL_HZ)

    finally:
        try:
            tello.send_rc_control(0, 0, 0, 0)
        except Exception:
            pass

        try:
            if flying:
                tello.land()
        except Exception:
            pass

        try:
            tello.closseVideo()
        except Exception:
            pass

        try:
            tello.__del__()
        except Exception:
            pass

        pygame.quit()
        cv2.destroyAllWindows()
        print("드론 연결 해제")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("프로그램 종료")