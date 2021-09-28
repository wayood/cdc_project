#!/usr/bin/env python
## coding: UTF-8

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from darknet_ros_msgs.msg import BoundingBoxes
#from std_msgs.msg import String
from ros_lecture_msgs.msg import Custom2
from sync_sub.msg import some_position

class ColorExtract(object):
    def __init__(self):
        self._vel_pub = rospy.Publisher('/dtw_robot1/diff_drive_controller/cmd_vel', Twist, queue_size=10)
        self._blue_pub = rospy.Publisher('blue_image', Image, queue_size=20)
        self._red_pub = rospy.Publisher('red_image', Image, queue_size=20)
        self._area_pub = rospy.Publisher('chatter2', some_position, queue_size=10)
        self._image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.callback)
        self._yolo_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes , self.call)
        self._bridge = CvBridge()
        self._vel = Twist()
        self._msg = some_position()

    def get_colored_area(self, cv_image, lower, upper):
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask_image = cv2.inRange(hsv_image, lower, upper)
        extracted_image = cv2.bitwise_and(cv_image, cv_image, mask=mask_image)
        area = cv2.countNonZero(mask_image)
        return (area, extracted_image)

    def callback(self, data):
        try:
            cv_image = self._bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError, e:
            print e
        self._blue_area, blue_image = self.get_colored_area(cv_image, np.array([50,100,100]), np.array([150,255,255]))
        self._red_area, red_image = self.get_colored_area(cv_image, np.array([0,150,100]), np.array([0,255,255]))
        try:
            self._blue_pub.publish(self._bridge.cv2_to_imgmsg(blue_image, 'bgr8'))
            self._red_pub.publish(self._bridge.cv2_to_imgmsg(red_image, 'bgr8'))
        except CvBridgeError, e:
            print e
        rospy.loginfo('blue=%d, red=%d' % (self._blue_area, self._red_area))
        self._msg.header.stamp = rospy.Time.now()
        self._msg.position = [self._blue_area,self._red_area,1]
        self._area_pub.publish(self._msg)
        #r.sleep()

    def call(self, data):
        for box in data.bounding_boxes:
            rospy.loginfo(box.Class)
            #rospy.loginfo(data)
            if box.Class == "traffic light":
                #rospy.loginfo("test_stop!")
                rospy.loginfo("11111111111!")
                if self._red_area > 500:
                    rospy.loginfo("2222222222!")
                    self._vel.linear.x = 0
                    self._vel_pub.publish(self._vel)

if __name__ == '__main__':
    rospy.init_node('color_extract')
    color = ColorExtract()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass