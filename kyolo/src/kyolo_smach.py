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



def child_term_cb(outcome_map):
    print("child_term_cb")
    if outcome_map['MOVE'] == 'succeeded':
        return True
    if outcome_map['MONITOR']:
        return True
    return False

def out_cb(outcome_map):
    print("out_cb")
    if outcome_map['MOVE'] == 'succeeded':
        return 'done'
    else:
        return 'exit'

def monitor_cb(ud, msg):
    print("monitor_cb")
    return False
         

def main():
    rospy.init_node('smach_example_state_machine')

    # Create the top level SMACH state machine
    sm_top = smach.StateMachine(outcomes=['done', 'exit'])
    
    # Open the container
    with sm_top:
        goal1=MoveBaseGoal()
        goal1.target_pose.header.frame_id = "dtw_robot1/map"
        goal1.target_pose.pose.position.x = 1.3
        goal1.target_pose.pose.position.y = -0.5
        goal1.target_pose.pose.orientation.w = 1.0
        smach.StateMachine.add('MOVE1', smach_ros.SimpleActionState('/dtw_robot1/move_base', MoveBaseAction, goal=goal1), transitions={'succeeded':'MOVE2', 'preempted':'MOVE1', 'aborted':'MOVE4'})

        goal2=MoveBaseGoal()
        goal2.target_pose.header.frame_id = "dtw_robot1/map"
        goal2.target_pose.pose.position.x = 1.7
        goal2.target_pose.pose.position.y = 0.6
        goal2.target_pose.pose.orientation.w = 1.0
        smach.StateMachine.add('MOVE2', smach_ros.SimpleActionState('/dtw_robot1/move_base', MoveBaseAction, goal=goal2), transitions={'succeeded':'MOVE3', 'preempted':'MOVE4', 'aborted':'MOVE4'})
         
        task2_concurrence = smach.Concurrence(outcomes=['done', 'exit'], default_outcome='done', child_termination_cb = child_term_cb, outcome_cb = out_cb)
        with task2_concurrence:
            goal=MoveBaseGoal()
            goal.target_pose.header.frame_id = "dtw_robot1/map"
            goal.target_pose.pose.position.x = 1.7
            goal.target_pose.pose.orientation.w = 2.3
            smach.Concurrence.add('MOVE', smach_ros.SimpleActionState('/dtw_robot1/move_base', MoveBaseAction, goal=goal))
            smach.Concurrence.add('MONITOR', smach_ros.MonitorState("/sm_stop", Empty, child_term_cb))
        smach.StateMachine.add('TASK2', task2_concurrence, transitions={'done':'MOVE3', 'exit':'exit'}) 

        goal3=MoveBaseGoal()
        goal3.target_pose.header.frame_id = "dtw_robot1/map"
        goal3.target_pose.pose.position.x = 0.7
        goal3.target_pose.pose.position.y = 2.5
        goal3.target_pose.pose.orientation.w = 1.0
        smach.StateMachine.add('MOVE3', smach_ros.SimpleActionState('/dtw_robot1/move_base', MoveBaseAction, goal=goal3), transitions={'succeeded':'MOVE4', 'preempted':'MOVE1', 'aborted':'MOVE2'})

       
        goal4=MoveBaseGoal()
        goal4.target_pose.header.frame_id = "dtw_robot1/map"
        goal4.target_pose.pose.position.x = 0.5
        goal4.target_pose.pose.position.y = 0.58
        goal4.target_pose.pose.orientation.w = 1.0
        smach.StateMachine.add('MOVE4', smach_ros.SimpleActionState('/dtw_robot1/move_base', MoveBaseAction, goal=goal4), transitions={'succeeded':'exit', 'preempted':'MOVE2', 'aborted':'MOVE3'})

        
        

    # Execute SMACH plan
    sis = smach_ros.IntrospectionServer('smach_server', sm_top, '/SM_ROOT')
    sis.start()
    outcome = sm_top.execute()
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
