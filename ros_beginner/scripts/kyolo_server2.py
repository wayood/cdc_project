#! /usr/bin/env python

import rospy
from ros_beginner.srv import Kyolo,KyoloResponse
from ros_beginner.msg import Kpub
from sync_sub.msg import some_position
from sync_sub.msg import some_position2
import message_filters
from geometry_msgs.msg import Twist

def callback(msg1, msg2):
  # do something
  position1 = msg1.Class
  position2 = msg2.position

  
  out = [position1, position2[0],position2[1]]
  #print(out)
  #out = [Class,blue_area,red_area]
  msg = Kpub()
  msg.position = position2
  msg.Class = position1
  #rospy.loginfo(msg)
  pub.publish(msg)


  if position1 == "traffic light" and position2[1] > 600:
            #rospy.loginfo("test_stop!")
            vel.linear.x = 0
            vel_pub.publish(vel)
  if position1 == "traffic light" and position2[0] > 600:
            #rospy.loginfo("test_Advance!!!!!")
            vel.linear.x = 1
            vel_pub.publish(vel)
  #else:
  #    a=1

def calculate(request):
    rospy.loginfo('called!')
    rospy.loginfo('Detect start...')

    position =[1,1,1]
    Class = 'Process Started'#'Process End!!'#'Detect Started!!'
    pub = rospy.Publisher('chatter6', some_position, queue_size=10)
        
    r = rospy.Rate(10) # 10hz
    #while not rospy.is_shutdown():
    for num in range(10):
        msg = some_position()

        msg.header.stamp = rospy.Time.now()

        msg.position = [7,8,9]
        pub.publish(msg)
        r.sleep()

    ts = message_filters.ApproximateTimeSynchronizer([sub1,sub2], 10, delay)
    ts.registerCallback(callback)
    return KyoloResponse(position,Class)

rospy.init_node('kyolo_server')

service = rospy.Service('bbox', Kyolo , calculate)

sub1 = message_filters.Subscriber('chatter1', some_position2)
sub2 = message_filters.Subscriber('chatter2', some_position)
vel_pub = rospy.Publisher('/dtw_robot1/diff_drive_controller/cmd_vel', Twist, queue_size=10)
vel = Twist()
pub = rospy.Publisher('chatter5', Kpub, queue_size=10)
fps = 10.
delay = 1/fps
#print(service)
#rospy.loginfo(service)
#if (a==1):
#    break
rospy.spin()