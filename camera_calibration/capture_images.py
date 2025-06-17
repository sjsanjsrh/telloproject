import cv2
import os
import time
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from telloController import TelloController

# 저장할 디렉토리
save_dir = "./calib_images"
os.makedirs(save_dir, exist_ok=True)

exit_code = False

print("▶ 's' 키: 이미지 저장 | 'q' 키: 종료")
i = 0
def frame_callback(frame):
    global i, exit_code
    if frame is None:
        time.sleep(0.05)
        return

    cv2.imshow("Tello Capture", frame)
    key = cv2.waitKey(1)
    if key == ord('s'):
        filename = os.path.join(save_dir, f"image_{i:02d}.png")
        cv2.imwrite(filename, frame)
        print(f"💾 저장됨: {filename}")
        i += 1
    elif key == ord('q'):
        exit_code = True
        return

tello = TelloController()
tello.start(motor_on=False)
tello.set_video_bitrate(tello.BITRATE_2MBPS)
tello.set_video_fps(tello.FPS_30)
tello.set_video_resolution(tello.RESOLUTION_480P)
tello.setUpVideo(show_video=False, camera_direction=TelloController.CAMERA_DOWNWARD, frame_callback=frame_callback)

print("드론 연결 성공")
tello.printInfo()
print("프레임 대기")

try:
    while not exit_code:
        time.sleep(0.1)
except KeyboardInterrupt:
    print("프로그램 종료")
finally:
    try:
            tello.land()
    except:
        pass
    tello.__del__()
    print("드론 연결 해제")