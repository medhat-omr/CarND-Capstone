from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        self._wheel_base = wheel_base
        self._steer_ratio = steer_ratio
        self._max_lat_accel = max_lat_accel
        self._max_steer_angle = max_steer_angle

    def control(self, trgt_lin_vel, trgt_ang_vel, curr_lin_vel, dbw_enabled):

        # If dbw was disabled
        if not dbw_enabled:
            return 0., 0., 0.

        yaw_controller = YawController(self._wheel_base, self._steer_ratio, ONE_MPH,
                                       self._max_lat_accel, self._max_steer_angle)
        steer = yaw_controller.get_steering(trgt_lin_vel, trgt_ang_vel, curr_lin_vel)

        # TODO: Implement all Constraints on Acceleration / Deceleration / Jerk ... etc. using PID, Lowpass Filter etc.

        return 1.0, 0.0, steer
