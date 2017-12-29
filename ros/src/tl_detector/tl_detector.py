#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import yaml

RED_COUNT_THRESHOLD = 1
YEL_COUNT_THRESHOLD = 5
GRN_COUNT_THRESHOLD = 5
UNK_COUNT_THRESHOLD = 50

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.world_2_car = None
        self.car_2_world = None

        self.waypoints = None
        self.camera_image = None
        self.lights = []

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        rospy.Subscriber('/image_color', Image, self.image_cb)

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

        pos_translation = [
            self.pose.pose.position.x,
            self.pose.pose.position.y,
            self.pose.pose.position.z
        ]

        pos_quaternion = [
            self.pose.pose.orientation.x,
            self.pose.pose.orientation.y,
            self.pose.pose.orientation.z,
            self.pose.pose.orientation.w
        ]

        self.car_2_world = tf.transformations.compose_matrix(
            translate=pos_translation,
            angles=tf.transformations.euler_from_quaternion(pos_quaternion))

        self.world_2_car = np.linalg.inv(self.car_2_world)

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''

        if self.state == TrafficLight.RED:
            STATE_COUNT_THRESHOLD = RED_COUNT_THRESHOLD
        if self.state == TrafficLight.YELLOW:
            STATE_COUNT_THRESHOLD = YEL_COUNT_THRESHOLD        
        if self.state == TrafficLight.GREEN:
            STATE_COUNT_THRESHOLD = GRN_COUNT_THRESHOLD
        if self.state == TrafficLight.UNKNOWN:
            STATE_COUNT_THRESHOLD = UNK_COUNT_THRESHOLD
            
            
        if self.state != state:    
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_light_closest_to_car_in_front(self):
        """Identifies the closest traffic ling position in front of the car
        Args:
            lights: array of lights, containing point coordinates in
            world reference frame
        Returns:
            an element value from the list or None if nothing is found
        """
        if self.world_2_car is None:
            return None

        result = None

        # Return None for distant traffic lights to turn off classification
        # procedure, which is very resource intensive and causes latency
        # problems
        min_dist = 100.0

        for light in self.lights:
            pos_world = np.array([light.pose.pose.position.x,
                                  light.pose.pose.position.y,
                                  light.pose.pose.position.z,
                                  1.0])

            pos_car = self.world_2_car.dot(pos_world)
            in_front = pos_car[0] > 0
            if not in_front:
                continue

            dist = np.linalg.norm(pos_car[:3])
            if dist < min_dist:
                min_dist = dist
                result = light

        return result

    def get_closest_stopline_pos3d(self, light):
        """Returns 3D position of stop-line closest to the given traffic light
        Args:
            light: traffic light properties
        Returns:
            found stop-line 3D coordinate as numpy array or None if not found
        """
        if self.world_2_car is None:
            return None

        result = None
        min_dist = np.inf

        stop_line_positions = self.config['stop_line_positions']

        light_pos3d = np.array([light.pose.pose.position.x,
                                light.pose.pose.position.y,
                                light.pose.pose.position.z])


        for p in stop_line_positions:
            pos3d = np.array(p + [0.0])
            dist = np.linalg.norm(light_pos3d - pos3d)
            if dist < min_dist:
                min_dist = dist
                result = pos3d

        return result

    def get_closest_waypoint(self, position):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            position: 3D position to match a waypoint to as numpy array

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        if self.waypoints is None:
            return -1

        result = -1
        min_dist = np.inf
        for wp_index, wp in enumerate(self.waypoints.waypoints):
            wp_position = np.array([wp.pose.pose.position.x,
                                    wp.pose.pose.position.y,
                                    wp.pose.pose.position.z])

            dist = np.linalg.norm(wp_position - position)
            if dist < min_dist:
                min_dist = dist
                result = wp_index

        return result

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if self.camera_image is None:
            return TrafficLight.UNKNOWN

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        result = self.light_classifier.get_classification(cv_image)
        return result #light.state

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        light = self.get_light_closest_to_car_in_front()
        if light is not None:
            stopline_pos = self.get_closest_stopline_pos3d(light)
            if stopline_pos is not None:
                light_wp = self.get_closest_waypoint(stopline_pos)
                state = self.get_light_state(light)
                return light_wp, state

        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
