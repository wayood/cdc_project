#! /usr/bin/env python

import rospy
from ros_beginner.srv import Kyolo2,Kyolo2Response

from sync_sub.msg import some_position
from sync_sub.msg import some_position2

from geometry_msgs.msg import Twist

import math
from darknet_ros_msgs.msg import BoundingBoxes

class Integration(object):
    def __init__(self):
        #self._sub2 = rospy.Subscriber('chatter2', some_position , self.call)      #color
        #self._sub1 = rospy.Subscriber('chatter1', some_position2 ,self.callback)  #Class
        self._sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes ,self.callback)
        
        self._vel_pub = rospy.Publisher('/dtw_robot1/diff_drive_controller/cmd_vel', Twist, queue_size=10)
        self._vel = Twist()
        self._red_area = 0
        self._flag=True

        #rate = rospy.Rate(50)  IMPORTANT roop velocity
        #rate.sleep()           last

        
    
    def callback(self, data):
        #rospy.loginfo(data.Class)                              #0
        
        for box in data.bounding_boxes:
            rospy.loginfo(box.xmax)
            Velocity=(((4.00-(((box.xmin + box.xmax)*0.01)/2))/2)*0.25*math.pi)
            if box.Class == "bottle":
                rospy.loginfo("found_angular!!")
                self._vel.angular.z = Velocity
                self._vel_pub.publish(self._vel)            #2


            Width=(box.xmax-box.xmin)
            if box.Class == "bottle":
                if Width < 200:
                    rospy.loginfo("forward!!")
                    self._vel.linear.x = 0.1
                    self._vel_pub.publish(self._vel)
                elif 200 < Width < 220:
                    rospy.loginfo("slow_forward!!")
                    self._vel.linear.x = 0.03
                    self._vel_pub.publish(self._vel)
                elif Width >230:
                        rospy.loginfo("backward!!")
                        self._vel.linear.x = -0.03
                        self._vel_pub.publish(self._vel)

                        rospy.loginfo('tracking end!!!!')
                        rospy.sleep(10)
                        #self._vel.angular.z = -3
                        #self._flag=False
                        break

            #if self._flag==False:
            #    break

    #def call(self,msg1):
            #rospy.loginfo('red_area red_area red_area = %d' %(msg1.position[1]))  #3
    #    self._red_area = msg1.position[1]
            #rospy.loginfo(self._red_area)                                         #4
    

def calculate(request):
        rospy.loginfo('called!')
        rospy.loginfo('Tracking start...')
        position =[2,2,2]
        Class = 'Process Started'#'Process End!!'#'Detect Started!!'
        
        function = Integration()
        return Kyolo2Response(position,Class)

if __name__ == '__main__':
    rospy.init_node('B_srv')

    service = rospy.Service('bbox2', Kyolo2 , calculate)
    

    rospy.spin()