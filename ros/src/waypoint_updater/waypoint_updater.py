#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
import numpy as np
from scipy.spatial import KDTree
import math

#EN: at waypotin 6392 area, lag occurs, and then after stop light vehcile stops updating position...
# dropping rospy refresh rate down to 30 might be a fix


LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/current_velocity',TwistStamped, self.velocity_cb)

        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        self.current_waypoint_idx = rospy.Publisher('current_waypoint_idx', Int32, queue_size=1)


        # TODO: Add other member variables you need below
        self.decel_limit = rospy.get_param('~decel_limit', -5)
        self.pose = None
        self.waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.light_idx = None
        self.target_speed = 25 # in mph
        self.current_vel = None

        self.loop()

    def loop(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            rate.sleep()
            lane = Lane()
            if self.waypoints and self.pose and self.current_vel:
                closet_waypoint_index = self.get_closest_waypoint_idx()
                wp_length = len(self.waypoints.waypoints)
                looked_ahead_index = closet_waypoint_index+LOOKAHEAD_WPS
                wrap_around = looked_ahead_index - wp_length

                # generate default target speed profile:
                lane.waypoints = self.waypoints.waypoints[closet_waypoint_index:looked_ahead_index]

                if wrap_around > 0:
                    lane.waypoints = np.append(lane.waypoints, self.waypoints.waypoints[0:wrap_around])
                # set default vehicle speed:
                for waypoint in lane.waypoints:
                    waypoint.twist.twist.linear.x = self.target_speed

                # Check for light status:
                if self.light_idx != -1 and self.light_idx >= closet_waypoint_index and (self.light_idx <=looked_ahead_index or (wrap_around>0 and  self.light_idx <= wrap_around)):
                    # number of stopping points:
                    wp2stop = self.light_idx - closet_waypoint_index
                    # check the current speed and see how many waypoints are needed to come to 0 speed...
                    minWaypoints2stop = abs(self.current_vel/0.8*self.decel_limit)
                    if minWaypoints2stop > wp2stop:
                        # this will attempt to break hard, but possibly fail.
                        for waypoint in lane.waypoints[0:self.light_idx - closet_waypoint_index]:
                            waypoint.twist.twist.linear.x = 0
                    else:
                        # set last 5 waypoints to 0:
                        end_idx = self.light_idx - closet_waypoint_index
                        start_idx = end_idx-5
                        if start_idx < 0:
                            start_idx = 0
                        for waypoint in lane.waypoints[start_idx:end_idx]:
                            waypoint.twist.twist.linear.x = 0
                        # smooth deceleration up to last 5 waypoints:
                        if start_idx > 0:
                            for i in range(1, start_idx):
                                lane.waypoints[i].twist.twist.linear.x = lane.waypoints[i-1].twist.twist.linear.x - self.decel_limit

                self.final_waypoints_pub.publish(lane)
                self.current_waypoint_idx.publish(closet_waypoint_index)


    def get_closest_waypoint_idx(self):
        # short circuit out of here:
        if self.waypoint_tree is None:
            return -1
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.waypoint_tree.query([x,y],1)[1]

        #check if closest is ahead of behind the vehicle:
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx-1]

        #hyperplane:
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x,y])

        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)

        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)

        return closest_idx


    def velocity_cb(self, msg):
        self.current_vel = msg.twist.linear.x

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        if not self.waypoints_2d:
                self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
                self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        self.light_idx = msg.data

    def obstacle_cb(self, msg):
        pass

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
