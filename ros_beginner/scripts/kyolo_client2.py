#! /usr/bin/env python

import rospy
from ros_beginner.srv import Kyolo2

def call():
    rospy.loginfo('waiting service...')
    
    rospy.wait_for_service('bbox2')
    try:
        
        service = rospy.ServiceProxy('bbox2', Kyolo2)
        

        response = service() #client -> server -> client


    except rospy.ServiceException, e:
        print "Service call failed: %s" % e



    #sum = service(a,b)
    #print '->',sum.result

    print '->',response.Class
    #print '->',response.position     #1

if __name__ == "__main__":
    rospy.init_node('kyolo_client2')
    call()