import cv2
import os
import time
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from telloController import TelloController

tello = TelloController()
cam_modis_dw = False  # False: 전면카메라, True: 하단카메라

# 저장할 디렉토리
# save_dir = "./calib_images"
save_dir = "./yolo_seg/data/img"
os.makedirs(save_dir, exist_ok=True)

exit_code = False

print("▶ 's' 키: 이미지 저장 | 'q' 키: 종료 | 'a' 키: 카메라 스왑")
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
    elif key == ord('a'):
        global cam_modis_dw
        cam_modis_dw = not cam_modis_dw
        if not cam_modis_dw:
            print("전면카메라로 전환")
            tello.switch_video_direction(TelloController.CAMERA_FORWARD)
        else:
            print("하단카메라로 전환")
            tello.switch_video_direction(TelloController.CAMERA_DOWNWARD)
    elif key == ord('z'):
        if(tello.get_video_res() == 720):
            print("비디오 해상도를 480P로 변경")
            tello.set_video_resolution(tello.RESOLUTION_480P)
        else:
            print("비디오 해상도를 720P로 변경")
            tello.set_video_resolution(tello.RESOLUTION_720P)

tello.start(motor_on=False)

tello.set_video_bitrate(tello.BITRATE_AUTO)
tello.set_video_fps(tello.FPS_30)
tello.set_video_resolution(tello.RESOLUTION_480P)
if not cam_modis_dw:
    print("전면카메라로 전환")
    tello.setUpVideo(show_video=False, camera_direction=TelloController.CAMERA_FORWARD, frame_callback=frame_callback)
else:
    print("하단카메라로 전환")
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