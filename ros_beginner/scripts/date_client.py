#!/usr/bin/env python                                                           
import rospy
from ros_beginner.srv import DateTrigger

class Client():
    def __init__(self):
        self.call = rospy.ServiceProxy('date_call', DateTrigger)
        self.res = ""

    def call(self):
        for i in  range(11):
            print(i)
            rospy.sleep(0.1)
        self.res = self.call()
        print("~~~~~~~~~~~~~~~~~~~~~~")
        print(self.res)
        print("~~~~~~~~~~~~~~~~~~~~~~")
        rospy.sleep(1)

if __name__ == '__main__':
    rospy.init_node("date_client")
    rospy.wait_for_service("date_call")
    c = Client()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        c.call()
        rate.sleep()