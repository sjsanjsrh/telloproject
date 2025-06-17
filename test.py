from djitellopy import Tello
import time
import cv2

tello = Tello()
tello.connect()
tello.streamon()

while True:
    frame = tello.get_frame_read().frame

    # 화면에 표시
    cv2.imshow('Drone Object Detection', frame)

    # q 키를 누르면 루프 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break