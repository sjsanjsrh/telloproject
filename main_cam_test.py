from __future__ import annotations

import time

from landing_controller import LandingConfig, LandingController
from telloController import TelloController

def main():
	tello = TelloController()
	landing = LandingController(
		tello,
		LandingConfig(
			white_threshold=250,
			white_threshold_contour=170,
			target_error_threshold=0.1,
			target_error_threshold_contour=0.05,
			contour_height_cm=30,
			hold_time_sec=0.4,
		),
	)

	try:
		tello.start(motor_on=False)
		tello.set_video_bitrate(tello.BITRATE_1MBPS)
		tello.set_video_fps(tello.FPS_30)
		tello.set_video_resolution(tello.RESOLUTION_480P)
		landing.setup_video(show_video=True)
		tello.printInfo()
		while not tello.can_read_frame():
			time.sleep(0.1)
		print("downward video ready")
		tello.set_speed(70)
		if not tello.can_flight():
			return

		tello.takeoff()
		height = tello.get_height()
		print(f"current height: {height}cm")
		tello.go_xyz_speed(0, 0, 120 - height, 100)
		landing.run()
	except KeyboardInterrupt:
		print("program stopped")
	except Exception as exc:
		print(f"error: {exc}")
		try:
			tello.land()
		except Exception:
			pass
	finally:
		try:
			tello.land()
		except Exception:
			pass
		tello.__del__()
		print("tello disconnected")


if __name__ == "__main__":
	main()
