#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

from twist_controller import Controller

# TODO(when waypoint_updater is ready): remove this
from std_msgs.msg import Int32

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''
CMD_RATE = 50 # 50Hz

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        # TODO(when waypoint_updater control is ready): remove this
        rospy.Subscriber('/traffic_waypoint', Int32, self.__traffic_waypoint_cb)
        self.traffic_wp = -1

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        self.controller = Controller(wheel_base, steer_ratio, max_lat_accel,
                                     max_steer_angle)

        # Subscribe to all needed topics
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)

        self.dbw_enabled = False
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)

        self.curr_lin_vel = None
        self.curr_ang_vel = None
        self.trgt_lin_vel = None
        self.trgt_ang_vel = None

        self.loop()

    def loop(self):
        rate = rospy.Rate(CMD_RATE)
        while not rospy.is_shutdown():
            if self.curr_lin_vel is None \
                    or self.curr_ang_vel is None \
                    or self.trgt_lin_vel is None \
                    or self.trgt_ang_vel is None:
                continue

            # Get predicted throttle, brake, and steering using `twist_controller`
            throttle, brake, steer = self.controller.control(self.trgt_lin_vel,
                                                             self.trgt_ang_vel,
                                                             self.curr_lin_vel,
                                                             self.dbw_enabled)

            # TODO(when waypoint_updater control is ready): remove this
            if -1 != self.traffic_wp:
                brake = 100
                throttle = 0

            # You should only publish the control commands if dbw is enabled
            if self.dbw_enabled:
              self.publish(throttle, brake, steer)
            rate.sleep()

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)

    # Callback function for twist_cmd
    def twist_cb(self, twist_stamped_msg):
        self.trgt_lin_vel = twist_stamped_msg.twist.linear.x
        self.trgt_ang_vel = twist_stamped_msg.twist.angular.z

    # Callback function for current_velocity
    def current_velocity_cb(self, twist_stamped_msg):
        self.curr_lin_vel = twist_stamped_msg.twist.linear.x
        self.curr_ang_vel = twist_stamped_msg.twist.angular.z

    # Callback function for current_velocity
    def dbw_enabled_cb(self,bool_msg):
        self.dbw_enabled = bool_msg.data

    # TODO(when waypoint_updater control is ready): remove this
    def __traffic_waypoint_cb(self, traffic_wp):
        self.traffic_wp = int(traffic_wp.data)


if __name__ == '__main__':
    DBWNode()
