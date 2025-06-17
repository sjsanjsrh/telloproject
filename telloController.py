from djitellopy import Tello
from threading import *
import cv2, time

class TelloController(Tello):
    __frame_read = None
    __frame = None

    __motor_on = False
    __show_video = False
    __camera_direction = None

    thread_readFrame:Thread = None

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
            frame = self.__frame_read.frame
            if frame is not None:
                if self.__camera_direction == Tello.CAMERA_FORWARD:
                    self.__frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                elif self.__camera_direction == Tello.CAMERA_DOWNWARD:
                    self.__frame = frame
            else:
                continue

            if self.__frame_callback is not None:
                self.__frame_callback(frame)

            if self.__show_video:
                cv2.imshow("DroneCamera", self.__frame)
                cv2.waitKey(1)

    def __setupVideo(self, show_video=False, camera_direction=Tello.CAMERA_FORWARD):
        if self.thread_readFrame is not None and self.thread_readFrame.is_alive():
            self.closseVideo()
        self.__frame_read = None
        self.set_video_direction(camera_direction)
        self.__show_video = show_video
        self.streamon()

        while self.__frame_read == None:
            self.__frame_read = self.get_frame_read()
            time.sleep(0.01)
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