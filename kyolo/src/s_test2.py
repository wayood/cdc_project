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


        #self._yolo_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes,self._callback_lambda)

    def execute(self, userdata):
        rospy.loginfo('Executing state STATE1')
        rospy.sleep(2.0)
        rospy.loginfo("found_angular!!")
        
        self._vel.angular.z = 1
        self._vel_pub.publish(self._vel) 

        if self.counter < 7:
            self.counter += 1
            return 'done'
        else:
            return 'exit'

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
        goal1.target_pose.pose.position.x = 2.5
        goal1.target_pose.pose.position.y = 2.8
        goal1.target_pose.pose.orientation.w = 1.0
        smach.StateMachine.add('MOVE1', smach_ros.SimpleActionState('/dtw_robot1/move_base', MoveBaseAction, goal=goal1), transitions={'succeeded':'MOVE2', 'preempted':'MOVE1', 'aborted':'MOVE1'})

        goal2=MoveBaseGoal()
        goal2.target_pose.header.frame_id = "dtw_robot1/map"
        goal2.target_pose.pose.position.x = 4.0
        goal2.target_pose.pose.position.y = 3.8
        goal2.target_pose.pose.orientation.w = 1.0
        smach.StateMachine.add('MOVE2', smach_ros.SimpleActionState('/dtw_robot1/move_base', MoveBaseAction, goal=goal2), transitions={'succeeded':'MOVE', 'preempted':'exit', 'aborted':'exit'})

        goal=MoveBaseGoal()
        goal.target_pose.header.frame_id = "dtw_robot1/map"
        goal.target_pose.pose.position.x = 6.8
        goal.target_pose.pose.position.y = 1.9
        goal.target_pose.pose.orientation.w = 1.0
        smach.StateMachine.add('MOVE', smach_ros.SimpleActionState('/dtw_robot1/move_base', MoveBaseAction, goal=goal), transitions={'succeeded':'DETECT_AREA', 'preempted':'exit', 'aborted':'MOVE2'})
         
        sm_sub = smach.StateMachine(outcomes=['done'])
        with sm_sub:


            smach.StateMachine.add('Detect_range', State1(), transitions={'done':'Interval', 'exit':'done'})
            smach.StateMachine.add('Interval', State2(), transitions={'done':'Detect_range'})
        smach.StateMachine.add('DETECT_AREA', sm_sub, transitions={'done':'MOVE3'}) 

        goal3=MoveBaseGoal()
        goal3.target_pose.header.frame_id = "dtw_robot1/map"
        goal3.target_pose.pose.position.x = 7.0
        goal3.target_pose.pose.position.y = 0.0
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
