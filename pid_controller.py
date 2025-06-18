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
    X_KP = 1.5
    X_KI = 0.01
    X_KD = 0.0
    Y_KP = 1.5
    Y_KI = 0.01
    Y_KD = 0.0
    Z_KP = 1.5
    Z_KI = 0.0
    Z_KD = 0.0

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
        vx = int(self.pid_pitch(roll_speed, dt))
        vy = int(self.pid_roll(pitch_speed, dt))
        vz = int(self.pid_throttle(throttle_speed, dt))
        vyaw = int(self.pid_yaw(0, dt))  # yaw는 센서값이 없으므로 0 입력
        return vx, vy, vz, vyaw

    def compute_position_error(self, ex, ey, ez=0, setpoint_yaw=0, dt=0.01):
        self.pid_x.set_point = 0
        self.pid_y.set_point = 0
        self.pid_z.set_point = 0
        vx = int(self.pid_x(ex, dt))
        vy = int(self.pid_y(ey, dt))
        vz = int(self.pid_z(ez, dt))

        return vx, vy, vz
    
    def control_speed(self, left_right, forward_backward, yaw, throttle, dt=0.01):
        # PID 제어를 통해 속도 계산
        cx = int(self.pid_roll(left_right, dt))
        cy = int(self.pid_pitch(forward_backward, dt))
        cz = int(self.pid_throttle(throttle, dt))
        cyaw = int(self.pid_yaw(yaw, dt))
        # 드론에 RC 명령 전송
        self.drone.send_rc_control(cx, cy, cz, cyaw)

    def control_position(self, ex, ey, ez=0, setpoint_yaw=0, dt=0.01):
        vx, vy, vz = self.compute_position_error(ex, ey, ez, setpoint_yaw, dt)
        # cx, cy, cz, cyaw = self.compute_speed(vx, vy, vz, setpoint_yaw, dt)
        # self.drone.send_rc_control(cy, cx, cz, cyaw)
        self.drone.send_rc_control(vy, vx, vz, 0)
    
    def move_to(self, x, y, z=0, setpoint_yaw=0, error_threshold=1, hold_time=1.0):
        """
        드론을 (x, y, z) 위치로 이동시키는 함수
        :param x: 목표 x 좌표
        :param y: 목표 y 좌표
        :param z: 목표 z 좌표 (기본값: 0)
        :param setpoint_yaw: 목표 yaw 각도 (기본값: 0)
        :param error_threshold: 오차 허용 범위 (기본값: 1)
        :param hold_time: 목표 위치에서 대기할 시간 (기본값: 1.0초)
        """
        current_x = 0
        current_y = 0
        current_z = 0
        self.init_dt()
        while True:
            time.sleep(0.1)
            dt = self.compute_dt()
            if dt <= 0:
                time.sleep(0.01)
                continue
            # 현재 위치와 목표 위치의 오차 계산
            current_x += float(self.drone.get_speed_x()) * dt
            current_y += float(self.drone.get_speed_y()) * dt
            current_z += float(self.drone.get_speed_z()) * dt
            print(f"dt: {dt:.2f}")
            if abs(current_x - x) < error_threshold and abs(current_y - y) < error_threshold and abs(current_z - z) < error_threshold:
                hold_time -= dt
                if hold_time <= 0:
                    print(f"목표 위치에 도달: ({x}, {y}, {z})")
                    break
            print(f"현재 위치: ({current_x:.2f}, {current_y:.2f}, {current_z:.2f}) 목표 위치: ({x}, {y}, {z})")
            self.control_position(current_x - x, current_y - y, current_z - z, setpoint_yaw, dt)


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
