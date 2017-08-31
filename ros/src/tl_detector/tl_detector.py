#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
from traffic_light_config import config

STATE_COUNT_THRESHOLD = 3


class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights helps you acquire an accurate ground truth data source for the traffic light
        classifier, providing the location and current color state of all traffic lights in the
        simulator. This state can be used to generate classified images or subbed into your solution to
        help you work on another single component of the node. This topic won't be available when
        testing your solution in real life so don't rely on it in the final submission.
        '''
        sub6 = rospy.Subscriber('/camera/image_raw', Image, self.image_cb)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()

        self.x = None
        self.y = None
        self.counter = 0

        # FIXME: Only 4 traffic lights so far (the one that works with the sim)

        self.start_x_light = [1130.0, 1540.0, 2122.0, 2172.0]
        self.end_x_light = [1145.0, 1550.0, 2125.0, 2175.0]
        self.start_y_light = [1183.0, 1173.0, 1451.0, 1723.0]
        self.end_y_light = [1184.0, 1161.0, 1532.0, 1790.0]
        self.tol = 0.5

        self.crop_1_x = [300, 340, 320, 250]
        self.crop_2_x = [500, 480, 500, 680]
        self.crop_1_y = [200, 300, 200, 100]
        self.crop_2_y = [350, 450, 500, 500]

        rospy.spin()

    def pose_cb(self, msg):
        self.x = float(msg.pose.position.x)
        self.y = float(msg.pose.position.y)

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        if self.x is None:
            return
        l_id = self.near_light_id()
        if l_id is None:
            return
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            roi = img[self.crop_1_y[l_id]:self.crop_2_y[l_id], self.crop_1_x[l_id]:self.crop_2_x[l_id]]
            # cv2.imwrite(
            #     '/home/crised/sdcnd/term3/CarND-Capstone/pics2/' + str(self.counter) + '_' + str(self.x) + '_' + str(
            #         self.y) + '_' + str(l_id) + '.jpeg', roi)
            self.counter += 1
            reds = self.red_count_enough(roi)
            if reds:
                # rospy.logerr('got red light')
                # cv2.imwrite(
                #     '/home/crised/sdcnd/term3/CarND-Capstone/pics/' + str(self.counter) + '_' + str(self.x) + '_' + str(
                #         self.y) + '_' + str(l_id) + '_' + str(self.red_count(roi)) + '.jpeg', roi)
                # self.counter += 1
                self.upcoming_red_light_pub.publish(Int32(1))
            else:
                self.upcoming_red_light_pub.publish(Int32(-1))
        except CvBridgeError, e:
            rospy.logerr('error %s', e)

    def near_light_id(self):
        for i in range(len(self.start_x_light)):
            min_x = min(self.start_x_light[i], self.end_x_light[i])
            max_x = max(self.start_x_light[i], self.end_x_light[i])
            min_y = min(self.start_y_light[i], self.end_y_light[i])
            max_y = max(self.start_y_light[i], self.end_y_light[i])
            if min_x - self.tol < self.x < max_x + self.tol:
                if min_y - self.tol < self.y < max_y + self.tol:
                    return i
        return None

    # The quality must be set to Fantastic in the simulator!
    def red_count_enough(self, img):
        height, width, _ = img.shape
        count = 0
        for i in range(height):
            for j in range(width):
                # r = int(img[i][j][0])
                # g = int(img[i][j][1])
                # b = int(img[i][j][2])
                b = int(img[i][j][0])
                g = int(img[i][j][1])
                r = int(img[i][j][2])
                # if r > 220 and g < 70 and b < 50:
                # if b > 200 and g < 110 and r < 105:
                if b > 200 and g < 110 and r < 105:
                    count += 1
                    if count > 7:
                        rospy.logerr('red points: %s', count)
                        return True
        rospy.logerr('red points: %s', count)
        if count == 0:
            return False
        return True


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
