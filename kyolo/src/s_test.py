#!/usr/bin/env python

import rospy
import smach
import smach_ros
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Empty

from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import Header
from std_msgs.msg import String

from geometry_msgs.msg import Twist

import math
import time


class State1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done','exit'])
        self.counter = 0
        self._vel_pub = rospy.Publisher('/dtw_robot1/diff_drive_controller/cmd_vel', Twist, queue_size=10)
        self._vel = Twist()


        self._yolo_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes,self._callback_lambda)

    def execute(self, userdata):
        rospy.loginfo('Executing state STATE1')
        rospy.sleep(2.0)
        rospy.loginfo("found_angular!!")
        
        #self._vel.angular.z = 3
        #self._vel_pub.publish(self._vel) 

        rospy.spin()

    def callback(data, vel, vel_pub):

     for box in data.bounding_boxes:
        rospy.loginfo(
            "Xmin: {}, Xmax: {} Ymin: {}, Ymax: {}, Class:{}, Centroid1:{},Centroid2:{}".format(
                box.xmin, box.xmax, box.ymin, box.ymax, box.Class, (box.xmin+box.xmax)/2,(box.ymin+box.ymax)/2
            )
        )

        Velocity=(((4.00-(((box.xmin + box.xmax)*0.01)/2))/2)*0.25*math.pi)
        if box.Class == "bottle":
            rospy.loginfo("found_angular!!")
            vel.angular.z = Velocity
            vel_pub.publish(vel)
        elif box.Class == "traffic light":
            rospy.loginfo("test_stop!")
            vel.linear.x = 0
            vel_pub.publish(vel)

        
        rospy.loginfo("found_angular_Velocity:{}".format(Velocity))
        

        Width=(box.xmax-box.xmin)

        if box.Class == "bottle":
         if Width < 200:
             rospy.loginfo("forward!!")
             vel.linear.x = 0.1
             vel_pub.publish(vel)
         elif 200 < Width < 220:
              rospy.loginfo("sllow_forward!!")
              vel.linear.x = 0.03
              vel_pub.publish(vel)
         elif Width >230:
                rospy.loginfo("backward!!")
                vel.linear.x = -0.03
                vel_pub.publish(vel)
          

        rospy.loginfo(
            "Width:{}".format(box.xmax-box.xmin)
            )

        #if self.counter < 3:
        #    self.counter += 1
        #    return 'done'
        #else:
        #    return 'exit'

        if not finished:

            if 

            return 'done'

    def callback(self, data):


class State2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

    def execute(self, userdata):
        rospy.loginfo('Executing state STATE2')
        rospy.sleep(2.0)
        return 'done'

def main():
    rospy.init_node('smach_example_state_machine')
   
    # Create the top level SMACH state machine
    sm_top = smach.StateMachine(outcomes=['done', 'exit'])
    
    # Open the container
    with sm_top:
        goal1=MoveBaseGoal()
        goal1.target_pose.header.frame_id = "dtw_robot1/map"
        goal1.target_pose.pose.position.x = 2.7
        goal1.target_pose.pose.position.y = 2.8
        goal1.target_pose.pose.orientation.w = 1.0
        smach.StateMachine.add('MOVE1', smach_ros.SimpleActionState('/dtw_robot1/move_base', MoveBaseAction, goal=goal1), transitions={'succeeded':'MOVE2', 'preempted':'MOVE1', 'aborted':'MOVE1'})

        goal2=MoveBaseGoal()
        goal2.target_pose.header.frame_id = "dtw_robot1/map"
        goal2.target_pose.pose.position.x = 4.3
        goal2.target_pose.pose.position.y = 3.8
        goal2.target_pose.pose.orientation.w = 1.0
        smach.StateMachine.add('MOVE2', smach_ros.SimpleActionState('/dtw_robot1/move_base', MoveBaseAction, goal=goal2), transitions={'succeeded':'TASK2', 'preempted':'exit', 'aborted':'TASK2'})


         
        sm_sub = smach.StateMachine(outcomes=['done'])
        with sm_sub:
              
            

            #vel_pub = rospy.Publisher('/dtw_robot1/diff_drive_controller/cmd_vel', Twist, queue_size=10)
            #vel = Twist()
            #callback_lambda = lambda x: callback(x, vel, vel_pub)
            #goal=MoveBaseGoal()

            #goal.target_pose.header.frame_id = "dtw_robot1/map"
            #goal.target_pose.pose.position.x = 4.5
            #goal.target_pose.pose.orientation.w = 1.0
            #smach.Concurrence.add('MOVE', smach_ros.SimpleActionState('/dtw_robot1/move_base', MoveBaseAction, goal=goal))
            #vel_pub = rospy.Publisher('/dtw_robot1/diff_drive_controller/cmd_vel', Twist, queue_size=10)
            #vel = Twist()
            #callback_lambda = lambda x: callback(x, vel, vel_pub)
            #while not rospy.is_shutdown():
            # rospy.init_node('listener', anonymous=True)
            # rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes , callback_lambda)
            # rospy.spin()
            #smach.Concurrence.add('DETECTION', smach_ros.DetectionState("/dtw_robot1/diff_drive_controller/cmd_vel", Twist, callback))
            smach.StateMachine.add('STATE1', State1(), transitions={'done':'STATE2', 'exit':'done'})
            smach.StateMachine.add('STATE2', State2(), transitions={'done':'STATE1'})
        smach.StateMachine.add('TASK2', sm_sub, transitions={'done':'MOVE3'}) 

        goal3=MoveBaseGoal()
        goal3.target_pose.header.frame_id = "dtw_robot1/map"
        goal3.target_pose.pose.position.x = 4.5
        goal3.target_pose.pose.position.y = 1.7
        goal3.target_pose.pose.orientation.w = 1.0
        smach.StateMachine.add('MOVE3', smach_ros.SimpleActionState('/dtw_robot1/move_base', MoveBaseAction, goal=goal3), transitions={'succeeded':'exit', 'preempted':'MOVE1', 'aborted':'MOVE2'})

# Execute SMACH plan
    sis = smach_ros.IntrospectionServer('smach_server', sm_top, '/SM_ROOT')
    sis.start()
    outcome = sm_top.execute()
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
