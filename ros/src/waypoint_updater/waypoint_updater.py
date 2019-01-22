#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from scipy.spatial import KDTree

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

LOOKAHEAD_WPS = 200  # Number of waypoints we will publish. You can change this number
PUBLISH_RATE = 10
MAX_DECEL = 0.5
STOP_LINE_DELTA = 5  # A few waypoints so that car nose stops at the line


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
        self.stop_line_idx = -1
        self.current_pose = None
        self.current_velocity = None
        self.waypoints = None
        self.waypoints_2d = None
        self.waypoints_tree = None

        # Saves the subscriber so that it can unregister in its callback
        self.base_waypoints_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.loop()

    def loop(self):
        rate = rospy.Rate(PUBLISH_RATE)
        while not rospy.is_shutdown():
            if self.current_pose and self.waypoints_tree:
                self.publish_waypoints()

            rate.sleep()

    def pose_cb(self, msg):
        self.current_pose = msg.pose

    def velocity_cb(self, msg):
        vx = msg.twist.linear.x
        vy = msg.twist.linear.y
        self.current_velocity = math.sqrt(vx**2 + vy**2)

    def waypoints_cb(self, msg):
        self.waypoints = msg.waypoints

        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y]
                                 for waypoint in self.waypoints]
            self.waypoints_tree = KDTree(self.waypoints_2d)

        # Unsubscribe as we do not need the base waypoints anymore
        self.base_waypoints_sub.unregister()

        rospy.loginfo("Base waypoints data processed, unsubscribed from /base_waypoints")

    def traffic_cb(self, msg):
        # Callback for /traffic_waypoint message and return the stop line index data
        self.stop_line_idx = msg.data

    def closest_waypoint_idx(self):
        ego_position = [self.current_pose.position.x, self.current_pose.position.y]

        closest_idx = self.waypoints_tree.query(ego_position, 1)[1]

        closest_waypoint = np.array(self.waypoints_2d[closest_idx])
        previous_waypoint = np.array(self.waypoints_2d[closest_idx - 1])

        ego_position = np.array(ego_position)

        waypoint_vec = closest_waypoint - previous_waypoint
        ego_vec = ego_position - closest_waypoint

        # Computes direction
        val = np.dot(waypoint_vec, ego_vec)

        # Checks if the waypoint is behind
        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)

        return closest_idx

    def publish_waypoints(self):
        final_lane = self.generate_lane()
        self.final_waypoints_pub.publish(final_lane)

    def generate_lane(self):
        lane = Lane()
        closest_idx = self.closest_waypoint_idx()
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        base_waypoints = self.waypoints[closest_idx:farthest_idx]

        if self.stop_line_idx == -1 or (self.stop_line_idx >= farthest_idx):
            lane.waypoints = base_waypoints
        else:
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)
        rospy.loginfo("Closest stop line: [index=%d]", self.stop_line_idx)
        return lane

    def decelerate_waypoints(self, waypoints, closest_idx):
        temp = []
        for i, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose
            stop_idx = max(self.stop_line_idx-closest_idx-STOP_LINE_DELTA, 0)
            dist = self.distance(waypoints, i, stop_idx)
            vel = math.sqrt(2*MAX_DECEL*dist)

            if ((vel < 1.0) or ((self.stop_line_idx >= i) and (i >= stop_idx))):
                vel = 0.0

            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            temp.append(p)
        return temp

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0

        def dl(a, b): return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position,
                       waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
