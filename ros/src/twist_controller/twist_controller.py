from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter
#import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
MIN_SPEED = 0.0
KP = 0.1
KI = 0.0001
KD = 10
TIME_STEP = 0.1
MIN_VAL = 0.
MAX_VAL = 1.

class Controller(object):
    def __init__(self, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        # TODO: Implement
        self.yaw_controller = YawController(wheel_base, steer_ratio, MIN_SPEED, max_lat_accel, max_steer_angle)
        self.pid_controller = PID(KP, KI, KD, MIN_VAL, MAX_VAL)

    def control(self, linear_velocity, angular_velocity, current_velocity, dbw_enabled):
        if not dbw_enabled:
            self.pid_controller.reset()
            return 0., 0., 0.

        error = linear_velocity - current_velocity
        ctrl_val = self.pid_controller.step(error, TIME_STEP)

        #rospy.loginfo('Velocity error={:.3f}, PID control value={:.3f}'.format(error, ctrl_val))

        throttle = ctrl_val #1.0
        brake = 0.0
        steer = self.yaw_controller.get_steering(linear_velocity, angular_velocity, current_velocity)

        return throttle, brake, steer
