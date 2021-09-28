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
        
        self._vel_pub = rospy.Publisher('diff_drive_controller/cmd_vel', Twist, queue_size=10)
        self._vel = Twist()
        self._red_area = 0
        
        #rate = rospy.Rate(50)  IMPORTANT roop velocity
        #rate.sleep()           last

    
    def callback(self, data):
        #rospy.loginfo(data.Class)                              #0
        
        if data.Class == "traffic light":
            #rospy.loginfo("test_stop!")
            #rospy.loginfo("11111111111!")                      #1
            if self._red_area > 700:
                rospy.loginfo("STOP!")                   #2
                self._vel.linear.x = 0
                self._vel_pub.publish(self._vel)

    def call(self,msg1):
        #rospy.loginfo('red_area red_area red_area = %d' %(msg1.position[1]))  #3
        self._red_area = msg1.position[1]
        #rospy.loginfo(self._red_area)                                         #4
    

def calculate(request):
        rospy.loginfo('called!')
        rospy.loginfo('Detect start...')
        position =[1,1,1]
        Class = 'Process Started'#'Process End!!'#'Detect Started!!'
        
        #pub = rospy.Publisher('chatter6', some_position, queue_size=10)
        #r = rospy.Rate(10) # 10hz
        
        #for num in range(101):
        #    msg = some_position()
        #    msg.header.stamp = rospy.Time.now()
        #    msg.position = [7,8,9]
        #    pub.publish(msg)
            #r.sleep()

        function = Integration()
        return KyoloResponse(position,Class)

if __name__ == '__main__':
    rospy.init_node('A_srv')

    service = rospy.Service('bbox', Kyolo , calculate)
    

    rospy.spin()