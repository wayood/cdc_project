#! /usr/bin/env python

import rospy
from ros_beginner.srv import Kyolo

def call():
    rospy.loginfo('waiting service...')
    
    rospy.wait_for_service('bbox')
    try:
        
        service = rospy.ServiceProxy('bbox', Kyolo)
        

        response = service() #client -> server -> client


    except rospy.ServiceException, e:
        print "Service call failed: %s" % e



    #sum = service(a,b)
    #print '->',sum.result

    print '->',response.Class
    #print '->',response.position     #1

if __name__ == "__main__":
    rospy.init_node('kyolo_client')
    call()