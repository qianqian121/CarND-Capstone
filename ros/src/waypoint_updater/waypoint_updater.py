#!/usr/bin/env python

import rospy
from styx_msgs.msg import Lane, Waypoint, TrafficLight, TrafficLightArray
import copy
from geometry_msgs.msg import TwistStamped, PoseStamped
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

LOOKAHEAD_WPS = 200  # Number of waypoints we will publish. You can change this number
CRUISE = 0
CROSS = 1
STOP = 2
RUN = 3
CRUISE_SPEED = 12  # 12
CROSS_SPEED = 3
RUN_SPEED = 12
GO_STOP_N_WPS = 5
SPEED_THRESHOLD = 1
STOP_WPS = 100  # Fixed length stop waypoint


class WaypointUpdater(object):
    """The purpose of this node is to publish a fixed number of waypoints ahead
    of the vehicle with the correct target velocities, depending on traffic
    lights and obstacles."""

    def __init__(self):
        rospy.init_node('waypoint_updater', log_level=rospy.DEBUG)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_cb)

        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.init = False
        self.current_pose = None
        self.wps = None
        self.N = None  # number of waypoints
        self.final_wps = None
        self.speed = None

        # lights and light wps
        self.K = 4  # Number of crosses #FIXME: Update number of crosses
        self.light_wps_ids = []
        self.light_pos = [(1148.56, 1184.65), (1559.2, 1158.43), (2122.14, 1526.79), (2175.237, 1795.71),
                          (1493.29, 2947.67), (821.96, 2905.8), (161.76, 2303.82), (351.84, 1574.65)]
        # state logic
        self.state = CRUISE
        self.cross_id = None
        self.cross_wp = None
        self.car_wp = None

        # stop
        self.stop = False

        # camera light
        # camera.light
        self.cam_stop = False
        self.loop()

    def pose_cb(self, msg):
        self.current_pose = msg

    def waypoints_cb(self, msg):
        if self.init:
            return
        self.wps = msg
        self.N = len(self.wps.waypoints)
        for wp in self.wps.waypoints:
            wp.twist.twist.linear.x = CRUISE_SPEED  # set to 17 MPH, no more than this
        self.set_all_cross_wps()

    def current_cb(self, msg):
        self.speed = msg.twist.linear.x

    def traffic_cb(self, msg):
        if int(msg.data) == 1:
            self.cam_stop = True
            rospy.logerr('red light')
        else:
            rospy.logerr('green light')
            self.cam_stop = False
        if self.wps is not None and self.current_pose is not None:
            self.init = True

    def dist(self, a_x, a_y, b_x, b_y):
        return math.sqrt((a_x - b_x) ** 2 + (a_y - b_y) ** 2)

    def get_closest_wp_index(self, x, y):
        max_distance_so_far = 50000  # 50000 meters away
        best_i = None
        for i in range(self.N):
            wp = self.wps.waypoints[i]
            distance = self.dist(wp.pose.pose.position.x, wp.pose.pose.position.y, x, y)
            if distance < max_distance_so_far:
                max_distance_so_far = distance
                best_i = i
        return best_i

    def set_all_cross_wps(self):
        for i in range(self.K):
            light_pos_x, light_pos_y = self.light_pos[i]
            self.light_wps_ids.append(self.get_closest_wp_index(
                light_pos_x,
                light_pos_y))

    def get_approach_speeds(self):
        v0 = 2
        v = []
        for i in range(STOP_WPS):
            v.append(v0 - v0 * i * 0.2 / float(STOP_WPS - 1))
        return v

    def nearest_cross_id(self):
        for i in range(self.K):
            cross_wp = self.wps.waypoints[self.light_wps_ids[i]]
            distance_to_start_cross = self.dist(cross_wp.pose.pose.position.x,
                                                cross_wp.pose.pose.position.y,
                                                self.current_pose.pose.position.x,
                                                self.current_pose.pose.position.y)
            if distance_to_start_cross < 50:  # meters
                return i
        return None

    def cross_distance(self):
        return self.dist(self.cross_wp.pose.pose.position.x,
                         self.cross_wp.pose.pose.position.y,
                         self.current_pose.pose.position.x,
                         self.current_pose.pose.position.y)

    def update_state_values(self):
        if not self.init:
            return
        self.car_wp = self.get_closest_wp_index(self.current_pose.pose.position.x, self.current_pose.pose.position.y)
        self.cross_id = self.nearest_cross_id()
        if self.cross_id is not None:
            self.cross_wp = self.wps.waypoints[self.light_wps_ids[self.cross_id]]
        else:
            self.cross_wp = None

    def set_final_wps(self):
        wps = []
        if self.state == CRUISE:
            wps = self.wps.waypoints[
                  self.car_wp:self.car_wp + LOOKAHEAD_WPS]  # Don't forget modulo N
        elif self.state == CROSS:
            for i in range(LOOKAHEAD_WPS):  # cross mode ENDS at stop_wp
                wps.append(copy.deepcopy(self.wps.waypoints[self.car_wp + i]))
                wps[-1].twist.twist.linear.x = CROSS_SPEED
        elif self.state == STOP:
            speeds = self.get_approach_speeds()
            for i in range(STOP_WPS):
                wps.append(copy.deepcopy(self.wps.waypoints[self.car_wp + i]))
                wps[-1].twist.twist.linear.x = speeds[i]
                # rospy.logerr("Stop speed: %s ", wps[-1].twist.twist.linear.x)
            rospy.logerr("Stopped waypoints dictated")
        elif self.state == RUN:
            for i in range(LOOKAHEAD_WPS):  # cross mode ENDS at stop_wp
                wps.append(copy.deepcopy(self.wps.waypoints[self.car_wp + i]))
                wps[-1].twist.twist.linear.x = RUN_SPEED
        self.final_wps = wps

    def update_final_wps(self):
        if not self.init:
            return
        self.update_state_values()
        if self.state == CRUISE:
            if self.cross_id is not None:
                self.state = CROSS
                rospy.logerr("Switching from Cruise to Near Cross")
                return
        elif self.state == CROSS:
            # Add wps if going slow to stop_go_wp
            if self.cross_distance() < 7:  # If we pass this mark either stop or run. Never go back to cross.
                if not self.cam_stop:
                    self.state = RUN
                    rospy.logerr("Green light switching to RUN- take off!")
                    return
                else:
                    self.state = STOP
                    rospy.logerr("Switching to stop mode")
                    return
        elif self.state == STOP:
            if not self.cam_stop:
                rospy.logerr("Green light, takeoff!")
                self.state = RUN
                return
            if self.stop:
                # rospy.logerr("Stop mode, light id: %s", self.cross_id)
                return
            self.stop = True
        elif self.state == RUN:
            self.stop = False
            if self.cross_id is None:
                rospy.logerr("We passed the light, cruise now!")
                self.state = CRUISE
                return
        self.set_final_wps()

    def loop(self):
        """Publish the final waypoints."""
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.update_final_wps()
            if self.final_wps is not None:
                lane = Lane()
                lane.header.frame_id = '/world'
                lane.header.stamp = rospy.Time(0)
                lane.waypoints = self.final_wps
                self.final_waypoints_pub.publish(lane)
            rate.sleep()


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
