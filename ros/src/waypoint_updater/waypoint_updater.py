#!/usr/bin/env python

import rospy
import sys
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLightArray, TrafficLight
from std_msgs.msg import Int32

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
CMD_RATE = 50 # 50Hz
ONE_KPH = 1000.0 / 3600
# velocity in simulator is 40 km/h
# MAX_VELOCITY = 40 * ONE_KPH

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)  # Should be published only once
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_waypoint_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self._base_wps = None
        self._curr_pose = None
        self._traffic_wp_idx = None
        self._is_initialized = False

        # rospy.spin()
        self.loop()     # I am not sure if this is the most correct way instead of using spin() ?


    def loop(self):
        rate = rospy.Rate(CMD_RATE)
        result_final_wps = Lane()

        while not rospy.is_shutdown():
            if self._base_wps is None or self._curr_pose is None or self._traffic_wp_idx is None:
                continue

            # Step 1: find nearest base point and traffic light
            nearest_wp_idx = self.find_nearest_wp()
            idx = nearest_wp_idx % self._n_base_wps

            # Step 2: Form final waypoints msg
            MIN_STOP_DIST = 20
            vm = rospy.get_param('/waypoint_loader/velocity') * ONE_KPH * 0.95 # 5% safety factor to allow
                                                                               # for overshoot
            c = min(0.2, 2.0 / vm) # m/s / m
            v0 = self.curr_lin_vel

            result_final_wps.waypoints = []
            cross_stop_line = False
            case = 0

            x1 = -1.0
            x2 = -1.0
            xf = -10.0

            # no light
            if -1 == self._traffic_wp_idx:
                if v0 < vm:
                    x1 = (vm - v0) / c
                case = 0

            # red light,
            if -1 != self._traffic_wp_idx:
                xf = self.distance(self._base_wps, idx, self._traffic_wp_idx) - MIN_STOP_DIST
                # close - stop
                if (xf <= (v0 / c)):
                    case = 1

                # medium - speed up then stop
                if ((v0 / c) < xf <= ((vm - v0) / c + vm / c)):
                    x1 = (c * xf - v0) / (2 * c)
                    case = 2

                # far- speed up to max speed, then cruise, then stop
                if (xf > ((vm - v0) / c + vm / c)):
                    x1 = (vm - v0) / c
                    x2 = xf - vm / c
                    case = 3

            for i in xrange(nearest_wp_idx, nearest_wp_idx + LOOKAHEAD_WPS + 1):
                idx = i % self._n_base_wps
                result_final_wps.waypoints.append(self._base_wps[idx])

                x = self.distance(self._base_wps, nearest_wp_idx, i)
                dv = 0.2
                v = 0.0
                # no traffic light
                if case == 0:
                    if (x <= x1):
                        v = v0 + c * x
                        v = min(v + dv, vm)
                    else:
                        v = vm

                # red light, close
                if (case == 1):
                    if (x <= xf):
                        v = c * (xf - x)
                        v = max(v - dv, 0.0)
                    else:
                        v = 0

                # red light, medium
                if (case == 2):
                    if (x <= x1):
                        v = v0 + c * x
                        v = min(v + dv, vm)
                    elif (x1 < x <= xf):
                        v = c * (xf - x)
                        v = max(v - dv, 0.0)
                    else:
                        v = 0

                # red light, far
                if ( case == 3):
                    if (x <= x1):
                        v = v0 + c * x
                        v = min(v + dv, vm)
                    elif (x1 < x <= x2):
                        v = vm
                    elif (x2 < x <= xf):
                        v = vm - c * (x - x2)
                        v = max(v - dv, 0.0)
                    else:
                        v = 0

                    if idx == self._traffic_wp_idx:
                        cross_stop_line = True

                    if cross_stop_line:
                        v = 0

                self.set_waypoint_velocity(result_final_wps.waypoints, -1, v)

            # Step 4: publish to final_waypoints topic
            self.final_waypoints_pub.publish(result_final_wps)
            rate.sleep()


    def pose_cb(self, pose_stamped_msg):
        self._curr_pose = pose_stamped_msg.pose

    def waypoints_cb(self, lane_msg):
        if not self._is_initialized:
            self._base_wps = lane_msg.waypoints
            self._n_base_wps = len(self._base_wps)
            self._is_initialized = True

    def traffic_waypoint_cb(self, traffic_wp):
        self._traffic_wp_idx = int(traffic_wp.data)

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def euclidean_dist(self, a, b):
        return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)

    def distance(self, waypoints, wp1, wp2):
        dist = 0

        # To deal with circular indexing
        if wp2 < wp1:
            wp2 += self._n_base_wps

        for i in range(wp1, wp2+1):
            idx = i % self._n_base_wps

            dist += self.euclidean_dist(
                waypoints[wp1].pose.pose.position,
                waypoints[idx].pose.pose.position)

            wp1 = idx

        return dist

    # Callback function for current_velocity
    def current_velocity_cb(self, twist_stamped_msg):
        self.curr_lin_vel = twist_stamped_msg.twist.linear.x

    def find_nearest_wp(self):
        # Find the nearest waypoint
        # TODO: Try use a KD-Tree Data structure to store _base_wps according to position
        nearest_dist = sys.maxint
        nearest_idx = -1
        for i in range(self._n_base_wps):
            dist = self.euclidean_dist(self._curr_pose.position, self._base_wps[i].pose.pose.position)
            if dist < nearest_dist:
                nearest_dist = dist
                nearest_idx = i

        return nearest_idx


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
