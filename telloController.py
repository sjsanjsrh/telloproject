from djitellopy import Tello
from threading import *
import cv2, time
import numpy as np

class TelloController(Tello):
    __frame_read = None
    __frame = None

    __motor_on = False
    __show_video = False
    __camera_direction = None

    thread_readFrame:Thread = None
    __lock = Lock()

    __frame_callback = None

    def __del__(self):
        super().__del__()
        try:
            if self.__motor_on:
                self.turn_motor_off()
            self.streamoff()
        except:
            pass

    def start(self, motor_on=False):
        self.connect()
        self.__motor_on = motor_on
        try:
            if motor_on:
                self.turn_motor_on()
        except:
            pass

    def printInfo(self):
        print(f"현재 배터리: {self.get_battery()}%")
        print(f"현재 온도: {self.get_temperature()}")

    def can_flight(self) -> bool:
        if(self.get_battery() < 15):
            print("비행불가 - 배터리 부족")
            return False
        elif self.get_temperature() > self.get_highest_temperature():
            print("비행불가 - 온도가 너무 높음")
            return False
        return True
    
    def can_read_frame(self) -> bool:
        if self.__frame_read is None:
            return False
        if self.__frame_read.frame is None:
            return False
        if self.thread_readFrame is None or not self.thread_readFrame.is_alive():
            return False
        return True
    
    def __readFrame(self):
        # self.__frame = self.__frame_read.frame
        while True:
            frame = self.__frame_read.get_queued_frame()
            if frame is None:
                time.sleep(0.001)
                continue
            else:
                if self.__camera_direction == Tello.CAMERA_FORWARD:
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                elif self.__camera_direction == Tello.CAMERA_DOWNWARD:
                    frame = frame[0:np.shape(frame)[0]//3, :, :]

            with self.__lock:
                self.__frame = frame

            if self.__frame_callback is not None:
                self.__frame_callback(self.__frame)

            if self.__show_video:
                if not hasattr(self, 'priv_frame_timestamp'):
                    self.priv_frame_timestamp = time.time()
                dt = time.time() - self.priv_frame_timestamp
                self.priv_frame_timestamp = time.time()
                out_frame = self.__frame.copy()
                # FPS 계산
                if dt > 0:
                    fps = 1 / dt
                    cv2.putText(out_frame, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                else:
                    fps = 0
                    cv2.putText(out_frame, "FPS: N/A", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)   
                cv2.imshow("DroneCamera", out_frame)
                cv2.waitKey(1)

    def __setupVideo(self, show_video=False, camera_direction=Tello.CAMERA_FORWARD):
        if self.thread_readFrame is not None and self.thread_readFrame.is_alive():
            self.closseVideo()
        self.__frame_read = None
        self.set_video_direction(camera_direction)
        self.__show_video = show_video
        self.streamon()

        while self.__frame_read == None:
            try:
                self.__frame_read = self.get_frame_read(with_queue=True, max_queue_len=1)
                time.sleep(0.01)
            except Exception as e:
                print(f"오류 발생: {e}")
        print("Video stream is ready.")

        # Start frame read thread
        self.thread_readFrame = Thread(target=self.__readFrame, daemon=True)
        self.thread_readFrame.start()

    def set_video_direction(self, direction):
        self.__camera_direction = direction
        super().set_video_direction(self.__camera_direction)

    # __setupVideo thread processing warper
    def setUpVideo(self, show_video = False, camera_direction=Tello.CAMERA_FORWARD, frame_callback=None):
        self.__frame_callback = frame_callback
        Thread(target=self.__setupVideo, args=[show_video, camera_direction], daemon=True).start()

    def closseVideo(self):
        self.streamoff()
        if self.thread_readFrame is not None and self.thread_readFrame.is_alive():
            self.thread_readFrame.join()
        self.__frame_read = None
        cv2.destroyAllWindows()

    def get_attitude(self):
        """IMU 각도(pitch, roll, yaw) 반환"""
        return {
            'pitch': self.get_pitch(),
            'roll': self.get_roll(),
            'yaw': self.get_yaw()
        }