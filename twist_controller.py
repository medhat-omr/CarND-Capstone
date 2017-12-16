from yaw_controller import YawController
from pid import PID

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
CMD_RATE = 50


class Controller(object):
    def __init__(self, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        self._wheel_base = wheel_base
        self._steer_ratio = steer_ratio
        self._max_lat_accel = max_lat_accel
        self._max_steer_angle = max_steer_angle

    def control(self, trgt_lin_vel, trgt_ang_vel, curr_lin_vel, dbw_enabled):
	pid = PID(kp = 1.0, ki = 0.5, kd = 0.0, mn = -5.0, mx = 5.0)
	err = trgt_lin_vel - curr_lin_vel
	dt = 1.0 / CMD_RATE
	acc = pid.step(err, dt)

        # If dbw was disabled
        if not dbw_enabled:
	    pid.reset()	
            return 0., 0., 0.

        yaw_controller = YawController(self._wheel_base, self._steer_ratio, ONE_MPH,
                                       self._max_lat_accel, self._max_steer_angle)
        steer = yaw_controller.get_steering(trgt_lin_vel, trgt_ang_vel, curr_lin_vel)

	### May need to add a low pass filter to acc if jerk becomes and issue.

	throttle = 0.0
	brake = 0.0
	if acc  > 0.0:
	    throttle = acc / 5.0
	if acc <= 0.0:
	    # brake = acc * '~vehicle_mass' * '~wheel_radius' 1736.35*.2413
	    brake = -1.0 * acc * 1736.35 * .2413    

        return throttle, brake, steer
