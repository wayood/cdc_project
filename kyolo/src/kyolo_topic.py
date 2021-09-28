#!/usr/bin/env python

import message_filters
import rospy
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import Header
from std_msgs.msg import String
#from ros_lecture_msgs.msg import Custom2


from geometry_msgs.msg import Twist
import sys
import math
import time

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sync_sub.msg import some_position
from sync_sub.msg import some_position2

flag=True

def callback(data, vel, vel_pub,msg,pub,msg2,pub2):

    for box in data.bounding_boxes:
        rospy.loginfo(
            "Xmin: {}, Xmax: {} Ymin: {}, Ymax: {}, Class:{}, Centroid1:{},Centroid2:{}".format(
                box.xmin, box.xmax, box.ymin, box.ymax, box.Class, (box.xmin+box.xmax)/2,(box.ymin+box.ymax)/2
            )
        )
        #rospy.loginfo(box.Class) 

        #Velocity=(((4.00-(((box.xmin + box.xmax)*0.01)/2))/2)*0.25*math.pi)
        #if box.Class == "bottle" or "cup" or "clock":
        #    rospy.loginfo("found_angular!!")
        #    vel.angular.z = Velocity
        #    vel_pub.publish(vel)
        
        if box.Class == "traffic light" or "stop sign":
            #rospy.loginfo("test_stop!")
            rospy.loginfo("test_stop!")
            #vel.linear.x = 0
            #vel_pub.publish(vel)
            
            a=(box.xmin+box.xmax)/2
            b=(box.ymin+box.ymax)/2
            Width=(box.xmax-box.xmin)

            msg.header.stamp = rospy.Time.now()
            
            msg.position = [a,b,1]
            #msg.position = [7,7,7]
            pub.publish(msg)

            msg2.header.stamp = rospy.Time.now()
            msg2.Class = box.Class
            #msg.position = [7,7,7]
            pub2.publish(msg2)
                
        
        



def main():
    vel_pub = rospy.Publisher('/dtw_robot1/diff_drive_controller/cmd_vel', Twist, queue_size=10)
    vel = Twist()


    pub = rospy.Publisher('chatter3', some_position, queue_size=10)  #zahyou

    pub2 = rospy.Publisher('chatter1', some_position2, queue_size=10) #class
    #rospy.init_node('talker3', anonymous=True)


    #rospy.Subscriber('ros_lecture_msgs', Custom2 , queue_size=1)
    #area_sub = rospy.Subscriber('ros_lecture_msgs', Custom2, queue_size=1)
    #fps=10
    #delay=1/fps

    #ts = message_filters.ApproximateTimeSynchronizer([sub1,sub2],10,delay)
    #ts.callback_lambda = lambda x: callback(x, vel, vel_pub)
    msg = some_position()
    msg2 = some_position2()
    callback_lambda = lambda x: callback(x, vel, vel_pub,msg,pub,msg2,pub2)
    while not rospy.is_shutdown():
        rospy.init_node('listener', anonymous=True)
        #rospy.init_node('kyolo_topic')
        #sub1=rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes , callback_lambda)
        #sub2=rospy.Subscriber('ros_lecture_msgs', Custom2, callback_lambda)

        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes , callback_lambda)
        #rospy.Subscriber('ros_lecture_msgs', Custom2, callback_lambda)

        rospy.spin()
        




if __name__ == '__main__':
   
    try :
        main()
    except rospy.ROSInterruptException:
        pass