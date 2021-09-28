#! /usr/bin/env python

import rospy
from ros_beginner.srv import Kyolo,KyoloResponse
from ros_beginner.msg import Kpub
from sync_sub.msg import some_position
from sync_sub.msg import some_position2
import message_filters
from geometry_msgs.msg import Twist
from darknet_ros_msgs.msg import BoundingBoxes

def callback(msg1, msg2):
  #rospy.loginfo('blue=%d, red=%d' % (msg2.position[0],(msg2.position[1]))
  rospy.loginfo('red_area red_area red_area=%d',msg2.position[1])

  for box in msg1.bounding_boxes:
        rospy.loginfo(box.Class)
         #rospy.loginfo(data)
        if box.Class == "traffic light":
             #rospy.loginfo("test_stop!")
            rospy.loginfo("11111111111!")
            if msg2.position[1] > 1000:
                rospy.loginfo("2222222222!")
                vel.linear.x = 0
                vel_pub.publish(vel)
  # do something
  #rospy.loginfo(msg1.header)
  #rospy.loginfo(msg1.image_header)
  #rospy.loginfo(msg1.bounding_boxes.Class)
  
  #position1 = msg1.bounding_boxes

  #position1 = box.Class                  #1
  #position2 = msg2.position              #2
  #rospy.loginfo(box.Class)               #3
  #rospy.loginfo(msg2.position)           #4

  #out = [position1, position2[0],position2[1]]
  #print(out)
  #out = [Class,blue_area,red_area]

  #msg = Kpub()                           #1
  #msg.position = position2               #2
  #msg.Class = position1                  #3
  #rospy.loginfo(msg)
  #pub.publish(msg)                       #4


  #1 if position1 == "traffic light":#and position2[1] > 600:
   #2         rospy.loginfo("test_stop!")
    #3        vel.linear.x = 0
     #4       vel_pub.publish(vel)

  #if position1 == "traffic light" and position2[0] > 600:
   #         #rospy.loginfo("test_Advance!!!!!")
    #        vel.linear.x = 1
     #       vel_pub.publish(vel)
  #else:
  #    a=1

def calculate(request):
    rospy.loginfo('called!')
    rospy.loginfo('Detect start...')
    position =[1,1,1]
    Class = 'Process Started'#'Process End!!'#'Detect Started!!'
    pub = rospy.Publisher('chatter6', some_position, queue_size=10) 
    r = rospy.Rate(10) # 10hz
    for num in range(10):
        msg = some_position()
        msg.header.stamp = rospy.Time.now()
        msg.position = [7,8,9]
        pub.publish(msg)
        #r.sleep()

    ts = message_filters.ApproximateTimeSynchronizer([sub1,sub2], 10, delay)
    ts.registerCallback(callback)
    return KyoloResponse(position,Class)



rospy.init_node('kyolo_server')

service = rospy.Service('bbox', Kyolo , calculate)

sub1 = message_filters.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes)
sub2 = message_filters.Subscriber('chatter2', some_position)

vel_pub = rospy.Publisher('/dtw_robot1/diff_drive_controller/cmd_vel', Twist, queue_size=20)
vel = Twist()
pub = rospy.Publisher('chatter5', Kpub, queue_size=10)
fps = 200.
delay = 1/fps*0.5



#rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes , callback_lambda)
rospy.spin()