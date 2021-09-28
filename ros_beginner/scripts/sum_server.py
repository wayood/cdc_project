#! /usr/bin/env python

import rospy
from ros_beginner.srv import Cal,CalResponse

def calculate(request):
    rospy.loginfo('called!')
     #rospy.loginfo(request)
    #rospy.loginfo(request.a)
    #rospy.loginfo(request.b)
     #rospy.loginfo(request.words.split())
     #rospy.loginfo(len(request.words.split()))
    result = request.a + request.b
     #print result
    return CalResponse(result)


rospy.init_node('sum_server')

service = rospy.Service('sum', Cal, calculate)

#print(service)
#rospy.loginfo(service)
rospy.spin()