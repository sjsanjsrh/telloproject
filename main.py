from telloController import TelloController
import time

tello = TelloController()
tello.start()
tello.setUpVideo(show_video=True)

while True:
    tello.printInfo()
    time.sleep(1)