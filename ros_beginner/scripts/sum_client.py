#! /usr/bin/env python

import rospy
from ros_beginner.srv import Cal

def call():
    rospy.loginfo('waiting service...')
    
    rospy.wait_for_service('sum')
    try:
        
        service = rospy.ServiceProxy('sum', Cal)
        print('please input num a ...')
        a = int(input())
        print('please input num b ...')
        b = int(input())


        response = service(a,b) #client -> server -> client


    except rospy.ServiceException, e:
        print "Service call failed: %s" % e



    #sum = service(a,b)
    #print '->',sum.result
    print '->',response.result

if __name__ == "__main__":
    rospy.init_node('sum_client')
    call()