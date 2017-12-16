from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
MIN_SPEED = 0.0

class Controller(object):
    def __init__(self, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        # TODO: Implement
        self.yaw_controller = YawController(wheel_base, steer_ratio, MIN_SPEED, max_lat_accel, max_steer_angle)

    def control(self, linear_velocity, angular_velocity, current_velocity, dbw_enabled):
        if not dbw_enabled:
            return 0., 0., 0.

        throttle = 1.0
        brake = 0.0
        steer = self.yaw_controller.get_steering(linear_velocity, angular_velocity, current_velocity)

        return throttle, brake, steer
