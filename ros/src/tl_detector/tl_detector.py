#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from message_filters import ApproximateTimeSynchronizer, Subscriber
from cv_bridge import CvBridge
from scipy.spatial import KDTree
from light_classification.tl_classifier import TLClassifier
import cv2
import yaml
import os

STATE_COUNT_THRESHOLD = 3
SYNC_QUEUE_SIZE = 20

class TLDetector(object):
    def __init__(self):
        # rospy.init_node('tl_detector')
        rospy.init_node('tl_detector', log_level=rospy.DEBUG)

        # Load configuration
        config_string = rospy.get_param('/traffic_light_config')

        self.config = yaml.load(config_string)
        self.is_site = self.config['is_site']
        self.lookahead_waypoints = self.config['lookahead_waypoints']

        self.current_pose = None
        self.waypoints = None
        self.waypoints_tree = None
        self.ready = False
        self.stop_line_positions_idx = []
        self.camera_image = None
        self.lights = []
        self.light_state = TrafficLight.UNKNOWN
        self.light_waypoint_idx = -1
        self.light_state_count = 0
        self.bridge = CvBridge()
        self.light_classifier = None

        self.base_waypoints_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # Subscribers
        subscribers = [Subscriber('/vehicle/traffic_lights', TrafficLightArray),
                       Subscriber('/current_pose', PoseStamped)]
        
        # Check if we force the usage of the simulator light state, not available when on site
        if self.config['use_light_state'] and not self.is_site:
            rospy.logwarn('Classifier disabled, using simulator light state')
            # Note that we do not subscribe to the camera image
        else:
            # Setup the classifier
            self.light_classifier = TLClassifier(self.config)
            # When the classifier is enabled then the camera image needs to be in sync with the current_pose
            subscribers.append(Subscriber('/image_color', Image))
            # TODO Find out, this should greatly improve performance:
            # Subscriber('/image_color', Image, queue_size=1, buff_size=???)
            # buff_size should be greater than queue_size * avg_msg_byte_size
            
        self.synced_sub = ApproximateTimeSynchronizer(subscribers, SYNC_QUEUE_SIZE, 0.1)
        self.synced_sub.registerCallback(self.synced_data_cb)

        # Publisher
        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        rospy.spin()

    def synced_data_cb(self, lights_msg, pose_msg, image_msg = None):
        self.lights = lights_msg.lights
        self.current_pose = pose_msg.pose
        self.camera_image = image_msg

        if self.ready:
            light_waypoint_idx, light_state = self.process_traffic_lights()

            '''
            Publish upcoming red lights at camera frequency.
            Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
            of times till we start using it. Otherwise the previous stable state is
            used.
            '''
            if self.light_state != light_state:
                self.light_state_count = 0
                self.light_state = light_state
            elif self.light_state_count >= STATE_COUNT_THRESHOLD:
                light_waypoint_idx = light_waypoint_idx if light_state == TrafficLight.RED else -1
                self.light_waypoint_idx = light_waypoint_idx
                self.upcoming_red_light_pub.publish(Int32(light_waypoint_idx))
            else:
                self.upcoming_red_light_pub.publish(Int32(self.light_waypoint_idx))

            self.light_state_count += 1

    def waypoints_cb(self, msg):
        self.waypoints = msg.waypoints

        if self.waypoints_tree is None:
            waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in self.waypoints]
            self.waypoints_tree = KDTree(waypoints_2d)

            # Precomputes the stop line position index
            stop_line_positions = self.config['stop_line_positions']
            self.stop_line_positions_idx = [self.find_closest_waypoint_idx(stop_line_position) for stop_line_position in stop_line_positions]
            rospy.logdebug('Stop line positions: %s', stop_line_positions)
            rospy.logdebug('Stop line positions index: %s', self.stop_line_positions_idx)
            self.ready = True

        # Unsubscribe as we do not need the base waypoints anymore
        self.base_waypoints_sub.unregister()

        rospy.loginfo('Base waypoints data processed, unsubscribed from /base_waypoints')

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        
        camera_image = self.camera_image

        vehicle_position = [self.current_pose.position.x, self.current_pose.position.y]
        vehicle_idx = self.find_closest_waypoint_idx(vehicle_position)

        closest_light_state = TrafficLight.UNKNOWN
        closest_light, closest_light_idx = self.find_closest_light(vehicle_idx)
        
        if closest_light is not None:
            closest_light_state = self.get_light_state(closest_light, camera_image)

        return closest_light_idx, closest_light_state

    def find_closest_light(self, vehicle_idx):
        closest_light = None
        closest_light_idx = -1
        min_distance = len(self.waypoints)
        
        for light, stop_line_idx in zip(self.lights, self.stop_line_positions_idx):
            stop_line_distance = stop_line_idx - vehicle_idx
            if stop_line_distance >= 0 and stop_line_distance < self.lookahead_waypoints and stop_line_distance < min_distance:
                min_distance = stop_line_distance
                closest_light = light
                closest_light_idx = stop_line_idx

        return closest_light, closest_light_idx

    def find_closest_waypoint_idx(self, position):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose: Tuple with x, y coordinates

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        return self.waypoints_tree.query(position, 1)[1]

    def get_light_state(self, light, image_msg = None):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify
            image_msg (Image): The camera image  

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light_state = TrafficLight.UNKNOWN

        # If no classifier is available uses the light state
        if self.light_classifier is None:
            light_state = light.state
        elif image_msg is not None:
            image_rgb = self.bridge.imgmsg_to_cv2(image_msg, "rgb8")
            light_state = self.light_classifier.get_classification(image_rgb)

        return light_state

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
