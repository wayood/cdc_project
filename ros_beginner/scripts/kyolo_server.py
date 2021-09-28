#! /usr/bin/env python

import rospy
from ros_beginner.srv import Kyolo,KyoloResponse
from ros_beginner.msg import Kpub
from sync_sub.msg import some_position


def calculate(request):
    rospy.loginfo('called!')
    rospy.loginfo(request)
     #rospy.loginfo(request.a)
     #rospy.loginfo(request.b)
     #rospy.loginfo(request.words.split())
     #rospy.loginfo(len(request.words.split()))
     #result = request.a + request.b
    

    #position = [sub.position]
    position =[1,1,1]
    Class = 'Process End!!'#'Detect Started!!'
    #print sub.position
    #print sub.Class
    pub = rospy.Publisher('chatter3', some_position, queue_size=10)
        
    r = rospy.Rate(10) # 10hz
    #while not rospy.is_shutdown():
    for num in range(101):
        msg = some_position()

        msg.header.stamp = rospy.Time.now()

        msg.position = [7,8,9]
        pub.publish(msg)
        r.sleep()


    return KyoloResponse(position,Class)


rospy.init_node('bbox_server')

#sub = rospy.Subscriber('chatter5', Kpub)
#sub = rospy.Subscriber('chatter3', some_position)
#print(sub)

#position1=sub.position
#Class1=sub.Class

service = rospy.Service('bbox', Kyolo , calculate)



#print(service)
#rospy.loginfo(service)
rospy.spin()