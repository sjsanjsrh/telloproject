from simple_pid import PID
from djitellopy import Tello
import time

class DronePIDController:
    # PID 파라미터를 클래스 상수로 정의
    PITCH_KP = 0.5
    PITCH_KI = 0.01
    PITCH_KD = 0.0
    ROLL_KP = 0.5
    ROLL_KI = 0.01
    ROLL_KD = 0.0
    THROTTLE_KP = 0.5
    THROTTLE_KI = 0.01
    THROTTLE_KD = 0.0
    YAW_KP = 0.5
    YAW_KI = 0.01
    YAW_KD = 0.0
    OUTPUT_LIMITS = (-30, 30)
    X_KP = 1.4
    X_KI = 0.4
    X_KD = 1.2

    Y_KP = 1.4
    Y_KI = 0.4
    Y_KD = 1.2

    Z_KP = 0.5
    Z_KI = 0.1
    Z_KD = 0.1

    def __init__(self, drone:Tello):
        self.drone = drone
        self.pid_pitch = PID(self.PITCH_KP, self.PITCH_KI, self.PITCH_KD, setpoint=0, output_limits=self.OUTPUT_LIMITS)
        self.pid_roll = PID(self.ROLL_KP, self.ROLL_KI, self.ROLL_KD, setpoint=0, output_limits=self.OUTPUT_LIMITS)
        self.pid_throttle = PID(self.THROTTLE_KP, self.THROTTLE_KI, self.THROTTLE_KD, setpoint=0, output_limits=self.OUTPUT_LIMITS)
        self.pid_yaw = PID(self.YAW_KP, self.YAW_KI, self.YAW_KD, setpoint=0, output_limits=self.OUTPUT_LIMITS)
        self.pid_x = PID(self.X_KP, self.X_KI, self.X_KD, setpoint=0, output_limits=self.OUTPUT_LIMITS)
        self.pid_y = PID(self.Y_KP, self.Y_KI, self.Y_KD, setpoint=0, output_limits=self.OUTPUT_LIMITS)
        self.pid_z = PID(self.Z_KP, self.Z_KI, self.Z_KD, setpoint=0, output_limits=self.OUTPUT_LIMITS)
        self.elapsed_time = None

    def reset(self):
        self.pid_pitch.reset()
        self.pid_roll.reset()
        self.pid_throttle.reset()
        self.pid_yaw.reset()

    def compute_speed(self, x, y, z=0, setpoint_yaw=0, dt=0.01):
        # 드론에서 현재 속도 얻기
        pitch_speed = self.drone.get_speed_x()  # 전후
        roll_speed = self.drone.get_speed_y()   # 좌우
        throttle_speed = self.drone.get_speed_z()  # 상하
        # setpoint를 동적으로 적용
        self.pid_pitch.setpoint = x
        self.pid_roll.setpoint = y
        self.pid_throttle.setpoint = z
        self.pid_yaw.setpoint = setpoint_yaw
        # PID 연산
        vx = self.pid_pitch(roll_speed, dt)
        vy = self.pid_roll(pitch_speed, dt)
        vz = self.pid_throttle(throttle_speed, dt)
        vyaw = self.pid_yaw(0, dt)  # yaw는 센서값이 없으므로 0 입력
        return vx, vy, vz, vyaw

    def compute_position_error(self, ex, ey, ez=0, setpoint_yaw=0, dt=0.01):
        self.pid_x.set_point = 0
        self.pid_y.set_point = 0
        self.pid_z.set_point = 0
        vx = self.pid_x(ex, dt)
        vy = self.pid_y(ey, dt)
        vz = self.pid_z(ez, dt)

        return vx, vy, vz
    
    def control_speed(self, left_right, forward_backward, yaw, throttle, dt=0.01):
        # PID 제어를 통해 속도 계산
        cx = self.pid_roll(left_right, dt)
        cy = self.pid_pitch(forward_backward, dt)
        cz = self.pid_throttle(throttle, dt)
        cyaw = self.pid_yaw(yaw, dt)
        # 드론에 RC 명령 전송
        cx, cy, cz, cyaw = int(cx), int(cy), int(cz), int(cyaw)
        self.drone.send_rc_control(cx, cy, cz, cyaw)

    def control_position(self, ex, ey, ez=0, setpoint_yaw=0, dt=0.01):
        vx, vy, vz = self.compute_position_error(ex, ey, ez, setpoint_yaw, dt)
        # cx, cy, cz, cyaw = self.compute_speed(vx, vy, vz, setpoint_yaw, dt)
        # self.drone.send_rc_control(cy, cx, cz, cyaw)
        vx, vy, vz = int(vx), int(vy), int(vz)
        self.drone.send_rc_control(vy, vx, vz, 0)

    def compute_dt(self):
        if self.elapsed_time is None:
            self.elapsed_time = time.time()
            return 0.0
        current_time = time.time()
        dt = current_time - self.elapsed_time
        self.elapsed_time = current_time
        return dt
    
    def init_dt(self):
        self.elapsed_time = None
