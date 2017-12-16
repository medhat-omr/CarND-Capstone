#!/usr/bin/env python

import rospy
import sys
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLightArray, TrafficLight

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
CMD_RATE = 10 # 10Hz

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)  # Should be published only once
        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self._base_wps = None
        self._curr_pose = None
        self._traffic_wps = None
        self._n_traffic_wps = 0
        self._is_initialized = False

        # rospy.spin()
        self.loop()     # I am not sure if this is the most correct way instead of using spin() ?


    def loop(self):
        rate = rospy.Rate(CMD_RATE)
        result_final_wps = Lane()

        while not rospy.is_shutdown():
            if self._base_wps is None or self._curr_pose is None or self._traffic_wps is None:
                continue

            # Step 1: find nearest base point and traffic light
            nearest_wp_idx = self.find_nearest_wp()
            nearest_traffic_wp_idx = self.find_nearest_traffic_wp()

            traffic_light_state = self._traffic_wps.lights[nearest_traffic_wp_idx].state

            # Step 2: Form final waypoints msg
            stop_dist = \
                self.euclidean_dist(self._base_wps[nearest_wp_idx].pose.pose.position,
                                    self._traffic_wps.lights[nearest_traffic_wp_idx].pose.pose.position)

            print('Neartest WP Idx: {0} \t while Nearest Traffic WP Idx: {1}, \t number of Traffic Lights: {2} \
                  \t stop_dist: {3}'.format(
                  nearest_wp_idx, nearest_traffic_wp_idx, self._n_traffic_wps, stop_dist))

            result_final_wps.waypoints = []
            for i in range(nearest_wp_idx, nearest_wp_idx + LOOKAHEAD_WPS + 1):
                idx = i % self._n_base_wps
                result_final_wps.waypoints.append(self._base_wps[idx])

            # ToDO: Make it function in current car velocity
            MIN_STOP_DIST = 20 # Assuming intersection length to be 20m
            MAX_STOP_DIST = 40

            if traffic_light_state != TrafficLight.GREEN and MIN_STOP_DIST < stop_dist < MAX_STOP_DIST:
                for i in range(LOOKAHEAD_WPS):
                    self.set_waypoint_velocity(result_final_wps.waypoints, i, 0)
            else:
                for i in range(LOOKAHEAD_WPS):
                    self.set_waypoint_velocity(result_final_wps.waypoints, i, 20.0)

            # Step 3: publish to final_waypoints topic
            self.final_waypoints_pub.publish(result_final_wps)
            rate.sleep()


    def pose_cb(self, pose_stamped_msg):
        self._curr_pose = pose_stamped_msg.pose

    def waypoints_cb(self, lane_msg):
        if not self._is_initialized:
            self._base_wps = lane_msg.waypoints
            self._n_base_wps = len(self._base_wps)
            self._is_initialized = True

    def traffic_cb(self, traffic_wp_msg):
        """Callback for /traffic_waypoint message."""
        self._traffic_wps = traffic_wp_msg
        self._n_traffic_wps = len(self._traffic_wps.lights)
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def euclidean_dist(self, a, b):
        return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        for i in range(wp1, wp2+1):
            dist += self.euclidean_dist(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

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

    def find_nearest_traffic_wp(self):
        # Find the nearest traffic waypoint
        # TODO: Find the closest traffic waypoint in front of the car instead of the absolute closest
        nearest_dist = sys.maxint
        nearest_idx = -1
        for i in range(self._n_traffic_wps):
            dist = self.euclidean_dist(self._curr_pose.position, self._traffic_wps.lights[i].pose.pose.position)
            if dist < nearest_dist:
                nearest_dist = dist
                nearest_idx = i

        return nearest_idx


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
