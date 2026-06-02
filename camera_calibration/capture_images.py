import os
import sys
import time

import cv2

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from telloController import TelloController


tello = TelloController()
camera_direction = TelloController.CAMERA_FORWARD
exit_code = False
i = 0


def camera_name(direction):
    if direction == TelloController.CAMERA_DOWNWARD:
        return "downward"
    return "forward"


def resolution_value():
    return tello.get_video_res() or 480


def save_dir():
    if camera_direction == TelloController.CAMERA_DOWNWARD:
        dirname = "downward"
    else:
        dirname = f"{camera_name(camera_direction)}_{resolution_value()}p"
    return os.path.join(".", "calib_images", dirname)


def ensure_save_dir():
    path = save_dir()
    os.makedirs(path, exist_ok=True)
    return path


def current_profile():
    if camera_direction == TelloController.CAMERA_DOWNWARD:
        return "downward"
    return f"{camera_name(camera_direction)}_{resolution_value()}p"


def save_frame(frame):
    global i
    path = ensure_save_dir()
    filename = os.path.join(path, f"image_{i:02d}.png")
    cv2.imwrite(filename, frame)
    print(f"Saved [{current_profile()}]: {filename}")
    i += 1


def switch_camera():
    global camera_direction
    if camera_direction == TelloController.CAMERA_FORWARD:
        camera_direction = TelloController.CAMERA_DOWNWARD
    else:
        camera_direction = TelloController.CAMERA_FORWARD
    print(f"Switched camera: {camera_name(camera_direction)}")
    tello.switch_video_direction(camera_direction)
    ensure_save_dir()


def switch_resolution():
    if tello.get_video_res() == 720:
        print("Switched resolution: 480p")
        tello.set_video_resolution(tello.RESOLUTION_480P)
    else:
        print("Switched resolution: 720p")
        tello.set_video_resolution(tello.RESOLUTION_720P)
    ensure_save_dir()


def frame_callback(frame):
    global exit_code
    if frame is None:
        time.sleep(0.05)
        return

    cv2.imshow("Tello Capture", frame)
    key = cv2.waitKey(1)
    if key == ord("s"):
        save_frame(frame)
    elif key == ord("q"):
        exit_code = True
    elif key == ord("a"):
        switch_camera()
    elif key == ord("z"):
        switch_resolution()


print("'s' save | 'q' quit | 'a' switch camera | 'z' switch 480p/720p")

tello.start(motor_on=False)
tello.set_video_bitrate(tello.BITRATE_AUTO)
tello.set_video_fps(tello.FPS_30)
tello.set_video_resolution(tello.RESOLUTION_480P)
ensure_save_dir()
tello.setUpVideo(show_video=False, camera_direction=camera_direction, frame_callback=frame_callback)

print(f"Ready: {current_profile()}")
tello.printInfo()

try:
    while not exit_code:
        time.sleep(0.1)
except KeyboardInterrupt:
    print("Interrupted")
finally:
    try:
        tello.land()
    except Exception:
        pass
    tello.__del__()
    print("Disconnected")
