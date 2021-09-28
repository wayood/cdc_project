#! /usr/bin/env python
import rospy
from std_msgs.msg import String
from sync_sub.msg import some_position
from obstacle_detector.msg import Obstacles
from geometry_msgs.msg import Twist

a = 0
def callback(message,vel,vel_pub):
    
    for box in message.circles:
            #rospy.loginfo(box)
            #rospy.loginfo("X = %f" , box.center.x)
            #rospy.loginfo("Y = %f" , box.center.y)
            #rospy.loginfo("radius = %f" , box.radius)

            #rospy.loginfo("X + radius = %f" , box.center.x + box.radius)
            rospy.loginfo("-------------------------------------------")
            
    #rospy.loginfo("I heard %d", message.radius)
    #rospy.loginfo(message)

    #if 0 < box.center.x  < (box.radius+box.radius/2) and -(box.radius+box.radius/2) < box.center.y < (box.radius+box.radius/2)
    if 0 < box.center.x  < box.radius+box.radius/2:
        #rospy.loginfo("-----11111---------------------------------------------")
        if -(box.radius+box.radius/2) < box.center.y < box.radius+box.radius/2:
         #if -0.5 < box.center.y < 0.5:

         if box.center.x > 0:
            for i in range(5000):
                #a+=1
                rospy.loginfo(i)
                #vel.angular.z = 0.9
                vel.linear.x = 0
                vel_pub.publish(vel)
                #rospy.loginfo("turn left")
                #rospy.sleep(5.0)

                #vel.linear.x = 0
                #vel_pub.publish(vel)
            rospy.sleep(10.0)

    
    #if box.center.x < 0.05:
     #   if -(box.radius+box.radius/2) < box.center.y < box.radius+box.radius/2:
      #      vel.linear.x = 0
       #     vel_pub.publish(vel)
        #    rospy.loginfo("back!!!!")


def listener():

    vel_pub = rospy.Publisher('/diff_drive_controller/cmd_vel', Twist, queue_size=10)
    vel = Twist()
    callback_lambda = lambda x: callback(x, vel, vel_pub)
    
    rospy.init_node('obstacle', anonymous=True)
    
    #sub = rospy.Subscriber('chatter', String, callback)
    sub = rospy.Subscriber('tracked_obstacles', Obstacles , callback_lambda)
    #sub = rospy.Subscriber('raw_obstacles', CircleObstacle , callback)
    #rospy.loginfo(1)
    rospy.spin()

if __name__ == '__main__':
    listener()