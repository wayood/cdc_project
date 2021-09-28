#! /usr/bin/env python
# -*- coding: utf-8 -*-
 
import rospy
import cv2
import sys
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
 
class depth_estimater:
    WIDTH = 50
    HEIGHT = 25
 
    def __init__(self):
 
        rospy.init_node('depth_estimater', anonymous=True)
        self.bridge = CvBridge()
        self._color_image = 0
        self._depth_image = 0
        self._sub_rgb = rospy.Subscriber("camera/rgb/image_raw",Image,self.callback1)
        self._sub_depth = rospy.Subscriber("camera/depth/image_raw",Image,self.ImageCallback)
        #self.mf = message_filters.ApproximateTimeSynchronizer([sub_rgb, sub_depth], 100, 10.0)
        #self.mf.registerCallback(self.ImageCallback)
        self._vel_pub = rospy.Publisher('diff_drive_controller/cmd_vel', Twist, queue_size=10)
        self._vel = Twist()


    def callback1(self, rgb_data):
        self._color_image = self.bridge.imgmsg_to_cv2(rgb_data, 'passthrough')
 
    def ImageCallback(self, depth_data):
        try:
            
            self._depth_image = self.bridge.imgmsg_to_cv2(depth_data, 'passthrough')
        except CvBridgeError, e:
            rospy.logerr(e)
 
        self._color_image.flags.writeable = True
        self._color_image = cv2.cvtColor(self._color_image, cv2.COLOR_BGR2RGB) 
        h, w, c = self._color_image.shape
 
        x1 = (w / 2) - self.WIDTH
        x2 = (w / 2) + self.WIDTH
        y1 = (h / 2) - self.HEIGHT
        y2 = (h / 2) + self.HEIGHT
        sum = 0.0
 
        for i in range(y1, y2):
            for j in range(x1, x2):
                self._color_image.itemset((i, j, 0), 0)
                self._color_image.itemset((i, j, 1), 0)
                #color_image.itemset((100,100,2), 0)
 
                if self._depth_image.item(i,j) == self._depth_image.item(i,j):
                    sum += self._depth_image.item(i,j)
 
        ave = sum / ((self.WIDTH * 2) * (self.HEIGHT * 2))
        print("%f [m]" % ave)
        if ave < 0.7:
                rospy.loginfo("2222222222!")
                self._vel.linear.x = -0.01
                self._vel_pub.publish(self._vel)

        cv2.normalize(self._depth_image, self._depth_image, 0, 1, cv2.NORM_MINMAX)
        cv2.namedWindow("color_image")
        cv2.namedWindow("depth_image")
        cv2.imshow("color_image", self._color_image)
        cv2.imshow("depth_image", self._depth_image)
        cv2.waitKey(10)
 
if __name__ == '__main__':
    try:
        de = depth_estimater()
        rospy.spin()
    except rospy.ROSInterruptException: pass