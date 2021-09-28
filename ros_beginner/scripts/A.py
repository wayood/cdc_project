#! /usr/bin/env python

import rospy
from ros_beginner.srv import Kyolo,KyoloResponse

from sync_sub.msg import some_position
from sync_sub.msg import some_position2

from geometry_msgs.msg import Twist

class Integration(object):
    def __init__(self):
        self._sub2 = rospy.Subscriber('chatter2', some_position , self.call)
        self._sub1 = rospy.Subscriber('chatter1', some_position2 ,self.callback)
        
        self._vel_pub = rospy.Publisher('/dtw_robot1/diff_drive_controller/cmd_vel', Twist, queue_size=10)
        self._vel = Twist()
        self._red_area = 0
        
    
    def callback(self, data):
        rospy.loginfo(data.Class)
        
        if data.Class == "traffic light":
            #rospy.loginfo("test_stop!")
            rospy.loginfo("11111111111!")
            if self._red_area > 1000:
                rospy.loginfo("2222222222!")
                self._vel.linear.x = 0
                self._vel_pub.publish(self._vel)

    def call(self,msg1):
        rospy.loginfo('red_area red_area red_area = %d' %(msg1.position[1]))
        self._red_area = msg1.position[1]
        rospy.loginfo(self._red_area)
    

if __name__ == '__main__':
    rospy.init_node('kyolo_integration')

    function = Integration()

    rospy.spin()