from yaw_controller import YawController
from pid import PID

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

CMD_RATE = 50 # 50Hz
MIN_SPEED = ONE_MPH

class Controller(object):
    def __init__(self, wheel_base, steer_ratio, max_lat_accel, max_steer_angle,
                 vehicle_mass, wheel_radius, brake_deadband, decel_limit, accel_limit):
        self._vehicle_mass = vehicle_mass
        self._wheel_radius = wheel_radius
        self._accel_limit = accel_limit
        self._decel_limit = decel_limit
        self._brake_deadband = brake_deadband
        # create YawController and PID objects
        self.yaw_controller = YawController(wheel_base, steer_ratio, MIN_SPEED,
                                            max_lat_accel, max_steer_angle)
        self.pid = PID(kp=0.5, ki=0.5, kd=0.01, mn=self._decel_limit, mx=self._accel_limit)

    def control(self, trgt_lin_vel, trgt_ang_vel, curr_lin_vel, dbw_enabled):
        # If dbw was disabled
        if not dbw_enabled:
            self.pid.reset()
            return 0., 0., 0.

        steer = self.yaw_controller.get_steering(trgt_lin_vel, trgt_ang_vel, curr_lin_vel)

        err = trgt_lin_vel - curr_lin_vel
        dt = 1.0 / CMD_RATE
        acc = self.pid.step(err, dt)

        throttle = 0.0
        brake = 0.0

        if acc > 0.0:
            throttle = acc / self._accel_limit

        # if acc < 0:
        #     if -acc > self._brake_deadband: # decel > brake deadband
        #         brake = -acc * self._vehicle_mass * self._wheel_radius

        # This doesn't look to me as the correct way for applying deadband; may revisit this later
        if acc <= 0.0:
            brake = -1.0 * (acc - self._brake_deadband) * self._vehicle_mass * self._wheel_radius
            # why do we need this?!
            if (err**2 < 0.1):
                self.pid.reset()

        return throttle, brake, steer
