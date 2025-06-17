from telloController import TelloController
import time

tello = TelloController()
def main():
    tello.start(motor_on=False)
    tello.set_video_bitrate(tello.BITRATE_2MBPS)
    tello.set_video_fps(tello.FPS_30)
    tello.set_video_resolution(tello.RESOLUTION_480P)
    tello.setUpVideo(show_video=True, camera_direction=TelloController.CAMERA_FORWARD)
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
    finally:
        try:
            tello.land()
        except:
            pass
        tello.__del__()
        print("드론 연결 해제")