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
#def callback(data):
#    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)

flag=True

def callback(data, vel, vel_pub):
    #a=0

    for box in data.bounding_boxes:
        rospy.loginfo(
            "Xmin: {}, Xmax: {} Ymin: {}, Ymax: {}, Class:{}, Centroid1:{},Centroid2:{}".format(
                box.xmin, box.xmax, box.ymin, box.ymax, box.Class, (box.xmin+box.xmax)/2,(box.ymin+box.ymax)/2
            )
        )
        #if box.Class == "traffic light":
        #    rospy.loginfo("test_stop!")
        #    vel.linear.x = 0
        #    vel_pub.publish(vel)   


        

        Velocity=(((4.00-(((box.xmin + box.xmax)*0.01)/2))/2)*0.25*math.pi)
        if box.Class == "bottle" or "cup" or "clock":
            rospy.loginfo("found_angular!!")
            vel.angular.z = Velocity
            vel_pub.publish(vel)
        
        if box.Class == "traffic light" or "stop sign":
            rospy.loginfo("test_stop!")
            vel.linear.x = 0
            vel_pub.publish(vel)
              #time.sleep(0.5)   
        #else:
        #      rospy.loginfo("retry_angular!!")
        #      vel.angular.z = 0.075
        #      vel_pub.publish(vel)

        
        rospy.loginfo("found_angular_Velocity:{}".format(Velocity))
        #rospy.loginfo("TEST2:{}".format(
        #    ((4.00-(((box.xmin + box.xmax)*0.01)/2))/2)*0.25*math.pi)
        #)

        Width=(box.xmax-box.xmin)

        if box.Class == "bottle":
         if Width < 200:
             rospy.loginfo("forward!!")
             vel.linear.x = 0.1
             vel_pub.publish(vel)
         elif 200 < Width < 220:
              rospy.loginfo("sllow_forward!!")
              vel.linear.x = 0.03
              vel_pub.publish(vel)
         elif Width >230:
                rospy.loginfo("backward!!")
                vel.linear.x = -0.03
                vel_pub.publish(vel)

                rospy.loginfo('aaaaaaaaaaaaaaaaaaaaaaaaaaaaa')
                rospy.sleep(10)
                vel.angular.z = -3
                flag=False
                break
                #sys.exit()
                #a=5
             #   if a<3:
              #   a+=1
                
                
        #if a==3:
         #  rospy.loginfo("Break!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
          # break
        #if a==5:
         #break
          
        #rospy.loginfo("NUM = :{}".format(Custom2))
        #rospy.loginfo(Custom2)

        rospy.loginfo(
            "Width:{}".format(box.xmax-box.xmin)
            )



def main():
    vel_pub = rospy.Publisher('/dtw_robot1/diff_drive_controller/cmd_vel', Twist, queue_size=10)
    vel = Twist()
    #rospy.Subscriber('ros_lecture_msgs', Custom2 , queue_size=1)
    #area_sub = rospy.Subscriber('ros_lecture_msgs', Custom2, queue_size=1)
    #fps=10
    #delay=1/fps

    #ts = message_filters.ApproximateTimeSynchronizer([sub1,sub2],10,delay)
    #ts.callback_lambda = lambda x: callback(x, vel, vel_pub)

    callback_lambda = lambda x: callback(x, vel, vel_pub)
    while not rospy.is_shutdown():
        rospy.init_node('listener', anonymous=True)
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