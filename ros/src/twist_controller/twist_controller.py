from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

CMD_RATE = 10.0 # 10Hz
MIN_SPEED = 0.0
KP = 0.1
KI = 0.0001
KD = 10.0
MIN_VAL = -5.0
MAX_VAL = 5.0

class Controller(object):
    def __init__(self, wheel_base, steer_ratio, max_lat_accel, max_steer_angle,
                 vehicle_mass, wheel_radius, brake_deadband, accel_limit, decel_limit):

        self.yaw_controller = YawController(wheel_base, steer_ratio, MIN_SPEED, max_lat_accel, max_steer_angle)
        self.pid_controller = PID(KP, KI, KD, MIN_VAL, MAX_VAL)
        self._vehicle_mass = vehicle_mass
        self._wheel_radius = wheel_radius
        self._brake_deadband = brake_deadband
        self._accel_limit = accel_limit
        self._decel_limit = decel_limit

    def control(self, linear_velocity, angular_velocity, current_velocity, dbw_enabled):
        if not dbw_enabled:
            self.pid_controller.reset()
            return 0., 0., 0.

        steer = self.yaw_controller.get_steering(linear_velocity, angular_velocity, current_velocity)

        error = linear_velocity - current_velocity
        dt = 1.0 / CMD_RATE
        accel = self.pid_controller.step(error, dt)

        throttle = 0.0
        brake = 0.0

        if accel > 0.0:
            throttle = accel / MAX_VAL

        if accel < -self._brake_deadband:
            brake = -accel * self._vehicle_mass * self._wheel_radius

        return throttle, brake, steer
