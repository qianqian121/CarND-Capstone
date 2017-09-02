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
CRUISE_SPEED = 6  # 12
CROSS_SPEED = 3
RUN_SPEED = 8
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
        self.light_wps_ids = []
        self.light_pos = [(1148.56, 1184.65), (1559.2, 1158.43), (2122.14, 1526.79), (2175.237, 1795.71),
                          (1493.29, 2947.67), (821.96, 2905.8), (161.76, 2303.82), (351.84, 1574.65)]
        self.K = len(self.light_pos)  # Number of crosses

        self.start_x_light = [1130.0, 1540.0, 2115.0, 2170.0, 1480.0, 815.0, 155.0, 345.0]
        self.end_x_light = [1145.0, 1560.0, 2121.0, 2175.0, 1492.0, 821.0, 161.0, 351.0]

        # state logic
        self.state = CRUISE
        self.cross_id = None
        self.cross_wp = None
        self.car_wp = None

        # stop
        self.stop = False

        self.white_line_wp = [291, 748, 2020, 2562, 2600, 2600, 2600, 2600]

        # CV
        self.cam_stop = True  # start red
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
            # rospy.logerr('red light')
        else:
            # rospy.logerr('green light')
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

    # Fix exact stop waypoint for each traffic light
    def get_approach_speeds(self):
        # FIXME: Reviewer, please see there seems to be a bug here, no matter what decreasing speeds I choose:
        # `self.target_twist.twist.linear.x` in dbw node will always be the speed of the first value of this list.
        # I want to reach
        v0 = CROSS_SPEED
        v = []
        m = self.white_line_wp[self.cross_id] - self.car_wp
        # if m < 0:
        #     rospy.logerr(" BAD WHITE LINE WP")
        #     for i in range(STOP_WPS):
        #         v.append(v0 - v0 * i / float(STOP_WPS - 1))
        # else:
        #     for i in range(m):
        #         speed = v0 - v0 * i * 1.5 / float(m - 1)
        #         if speed < 0.1:
        #             speed = 0.0
        #         v.append(speed)
        #     for i in range(LOOKAHEAD_WPS - m):
        #         v.append(0.0)  # pad with 0s
        for i in range(m):
            v.append(0.0)  # Trick speed, to solve this 'bug'
        # rospy.logerr("STOP speeds: %s", str(v))
        return v

    def nearest_cross_id(self):
        for i in range(self.K):
            cross_wp = self.wps.waypoints[self.light_wps_ids[i]]
            distance_to_start_cross = self.dist(cross_wp.pose.pose.position.x,
                                                cross_wp.pose.pose.position.y,
                                                self.current_pose.pose.position.x,
                                                self.current_pose.pose.position.y)
            if distance_to_start_cross < 70:  # meters
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
            for i in range(len(speeds)):
                wps.append(copy.deepcopy(self.wps.waypoints[self.car_wp + i]))
                wps[-1].twist.twist.linear.x = speeds[i]
                # rospy.logerr("Stop speed: %s ", wps[-1].twist.twist.linear.x)
                # for wp in wps:
                #     rospy.logerr('stop wp speeds: : %s %s', wp.twist.twist.linear.x, wp.twist.twist.linear.y)
        elif self.state == RUN:
            for i in range(LOOKAHEAD_WPS):  # cross mode ENDS at stop_wp
                wps.append(copy.deepcopy(self.wps.waypoints[self.car_wp + i]))
                wps[-1].twist.twist.linear.x = RUN_SPEED
        self.final_wps = wps

    def in_camera_interval(self):
        if self.cross_id is None:
            rospy.logerr('what? no light id!')
        x = self.current_pose.pose.position.x
        margin = 1
        if self.speed < 0.5: # First light
            margin = 13
        if self.start_x_light[self.cross_id] + margin < x < self.end_x_light[self.cross_id]:
            rospy.logerr('In camera range for light: %s', self.cross_id)
            return True
        # rospy.logerr('Not in camera range')
        return False

    def update_final_wps(self):
        if not self.init:
            rospy.logerr('not init')
            return
        self.update_state_values()
        rospy.logerr('wp: %s', self.car_wp)
        if self.state == CRUISE:
            if self.cross_id is not None:
                self.state = CROSS
                rospy.logerr("Switching from Cruise to Near Cross")
                return
        elif self.state == CROSS:
            # Add wps if going slow to stop_go_wp
            # if self.cross_distance() < 7:  # If we pass this mark either stop or run. Never go back to cross.
            if self.in_camera_interval():  # If we pass this mark either stop or run. Never go back to cross.
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
