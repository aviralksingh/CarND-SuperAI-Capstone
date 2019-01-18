#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from scipy.spatial import KDTree
from tf.transformations import euler_from_quaternion

import math
import numpy as np

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
# TODO Check if this is ok
PUBLISH_RATE = 10
MAX_DECEL=0.5
STOP_DIS = 2.0

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
        self.stop_line_idx= -1
        self.towards_rad = None
        self.current_pose = None
        self.position = None
        self.current_velocity = None
        self.vel_base = 0.0
        self.waypoints = None
        self.waypoints_2d = None
        self.waypoints_tree = None
        self.stopping = False
        self.stop_dict = {}
        self.traffic_waypoint = None

        # Saves the subscriber so that it can unregister in its callback
        self.base_waypoints_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        # TODO Might be useful?
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb, queue_size=1)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        rate = rospy.Rate(PUBLISH_RATE)
        final_waypoints_exist = False
        while not rospy.is_shutdown():
            if self.current_pose and self.waypoints_tree:
            	
            	final_waypoints_exist = self.set_final_waypoints()

            if final_waypoints_exist:
                self.final_waypoints_pub.publish(self.final_waypoints)
            rate.sleep()


        rospy.spin()

                #closest_idx = self.closest_waypoint_idx()
                #final_waypoints = self.waypoints[closest_idx:closest_idx + LOOKAHEAD_WPS]

                # Call Publish_Waypoints
                #self.publish_waypoints()
                #lane = Lane()
                #lane.waypoints = final_waypoints
                #self.final_waypoints_pub.publish(lane)

            #rate.sleep()


    def pose_cb(self, msg):
        self.current_pose = msg.pose
        self.position = self.current_pose.position
        _x = self.current_pose.orientation.x
        _y = self.current_pose.orientation.y
        _z = self.current_pose.orientation.z
        _w = self.current_pose.orientation.w
        self.towards_rad = euler_from_quaternion([_x,_y,_z,_w])[2]

    def velocity_cb(self, msg):
        vx = msg.twist.linear.x
        vy = msg.twist.linear.y
        self.current_velocity = math.sqrt(vx**2 + vy**2)

    def waypoints_cb(self, waypoints):
        _max_vel = 0.0
        if self.waypoints == None:
            self.waypoints = []
            for idx, _waypoint in enumerate(waypoints.waypoints):
                _w = Waypoint()
                _w = _waypoint
                self.waypoints.append(_w)
                _vel = self.get_waypoint_velocity(_w)
                if _vel > _max_vel:
                    _max_vel = _vel
                    self.vel_base = self.get_waypoint_velocity(_w)

    def get_closest_waypoint(self):
        closest_dist = 999999.
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(len(self.waypoints)):
            dist = dl(self.position, self.waypoints[i].pose.pose.position)
            if dist < closest_dist:
                closest_dist = dist
                closest_waypoint = i
        return closest_waypoint

    def set_final_waypoints(self):
        final_waypoints_exist = False
        if (self.waypoints is not None) and (self.position is not None) and (self.traffic_waypoint is not None):

            final_waypoints_exist = True

            # Get closest waypoint idx
            closest_waypoint = self.get_closest_waypoint()

            # Get index of stopline
            stopline_waypoint_idx = self.traffic_waypoint
            stopline_waypoint_idx = int(str(stopline_waypoint_idx).split()[1])

            # Get distance to next stopline
            dist_next_stopline = self.distance(self.waypoints, closest_waypoint, stopline_waypoint_idx)
            dist_next_stopline = max(0, dist_next_stopline - STOP_DIS)

            # Initialize
            req_a = 0
            stop_time = 0
            stop_distance = 0

            # If ahead traffic light is red, then calcultate
            # Request acceleration [m/s^2]
            # Expected time to stop [sec]
            # Expected distance to stop [m]
            if dist_next_stopline != 0:
                req_a = -(self.current_velocity ** 2) / (2 * dist_next_stopline)
                stop_time = - (self.current_velocity / req_a)
                stop_distance = -(0.5 * req_a) * (stop_time ** 2) + (self.current_velocity * stop_time)

            # Scheduling the target velocity from stop signal on position to stopline
            if req_a < -0.7 and stop_distance != 0 and stop_distance > dist_next_stopline and self.stopping == False:
                self.stopping = True

                STOP_DIS = 1
                self.stop_dict = {}

                # Get waypoint index to stopline and assign velocity to each waypoint
                for i in range(0, stopline_waypoint_idx - closest_waypoint):

                    if stopline_waypoint_idx - closest_waypoint - STOP_DIS > 0:
                        vel_input = max(self.current_velocity - (self.current_velocity * (i+1) / (stopline_waypoint_idx - closest_waypoint - STOP_DIS)), 0)
                    else:
                        vel_input = 0

                    self.stop_dict[closest_waypoint + i] = vel_input
            #
            # # If vehicle req acceleration is less than max decel, than just go
            # if req_a < DECEL_LIMIT:
            #     self.stopping = False

            if stopline_waypoint_idx == -1:
                # If green light
                Is_red_light = False
                self.set_waypoint_velocity(self.waypoints, closest_waypoint, self.vel_base)
                self.stopping = False
            else:
                Is_red_light = True
                self.set_waypoint_velocity(self.waypoints, closest_waypoint, self.vel_base)

                if self.stopping == True:
                    self.set_waypoint_velocity(self.waypoints, closest_waypoint, self.stop_dict[closest_waypoint])

            _lookahead_wps = LOOKAHEAD_WPS
            if closest_waypoint + _lookahead_wps > len(self.waypoints):
                _lookahead_wps = len(self.waypoints) - closest_waypoint

            # set final waypoints
            self.final_waypoints = Lane()
            self.final_waypoints.waypoints = []
            for i in range(_lookahead_wps):

                self.final_waypoints.waypoints.append(self.waypoints[closest_waypoint + i])

        return final_waypoints_exist


    def traffic_cb(self, msg):
        #Callback for /traffic_waypoint message and return the stop line index data
        self.stop_line_idx=msg.data

    
    def publish_waypoints(self):
        final_lane=self.generate_lane()
        self.final_waypoints_pub.publish(final_lane)

    def generate_lane(self):
        lane=Lane()
        closest_idx= self.closest_waypoint_idx()
        farthest_idx = closest_idx +LOOKAHEAD_WPS
        base_waypoints= self.waypoints[closest_idx:farthest_idx]

        if self.stop_line_idx == -1 or (self.stop_line_idx >= farthest_idx):
            lane.waypoints = base_waypoints
            #lane.waypoints = self.decelerate_waypoints(base_waypoints,closest_idx)
        else:
            lane.waypoints = self.decelerate_waypoints(base_waypoints,closest_idx)
        rospy.loginfo("Closest stop line: [index=%d]", self.stop_line_idx)
        return lane


    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
