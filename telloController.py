from djitellopy import Tello
from threading import *
import cv2, time

class TelloController(Tello):
    __frame_read = None
    __frame = None

    __motor_on = False

    thread_readFrame:Thread
    thread_showCameraVideo:Thread        

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
        print(f"현재 배터리: {self.get_battery()}%\n")
        print(f"현재 온도: {self.get_temperature()}\n")

    def canFlight(self) -> bool:
        if(self.get_battery() < 15):
            print("비행불가 - 배터리 부족")
            return False
        elif self.get_temperature() > self.get_highest_temperature():
            print("비행불가 - 온도가 너무 높음")
            return False
        return True

    def __readFrame(self):
        self.__frame = self.__frame_read.frame
        time.sleep(0.001)

    def __showCameraVideo(self):
        cv2.imshow("DroneCamera", self.__frame)

    def __setupVideo(self, show_video=False):
        self.set_video_direction(Tello.CAMERA_DOWNWARD)
        
        # Wait 1 second until streamon
        self.streamon()
        time.sleep(2)

        # Start frame read thread
        self.__frame_read = self.get_frame_read()
        self.thread_readFrame = Thread(target=self.__readFrame, daemon=True)
        self.thread_readFrame.run()

        while self.__frame_read != None:
                time.sleep(0.01)

        # Displaying camera video on the screen using cv2
        if show_video:
            self.thread_showCameraVideo = Thread(target=self.__showCameraVideo, daemon=True)
            self.thread_readFrame.run()

    # __setupVideo thread processing warper
    def setUpVideo(self, show_video = False):
        Thread(target=self.__setupVideo, args=[show_video], daemon=True).run()
        