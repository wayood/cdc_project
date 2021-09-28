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

from ros_beginner.srv import Kyolo
import actionlib
from ros_beginner.msg import Practice2Action
from ros_beginner.msg import Practice2Goal



class Callback(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

    def execute(self, userdata):
        rospy.loginfo('Executing state YOLO START...')
        rospy.sleep(5.0)
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
        return 'done'

class State1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed','aborted'])
        self.counter = 0
        
    def execute(self, userdata):
        rospy.loginfo('Executing state DETECT START...')
        rospy.sleep(2.0)
        action_client = actionlib.SimpleActionClient( 'action', Practice2Action )
        action_client.wait_for_server() # Wait until the server is ready     
        # Set GoUntilBumperGaol's instance
        goal = Practice2Goal() 

        #goal.target_vel.linear.x = 1.0 #0.8
        goal.timeout_sec = 10
    
        action_client.send_goal( goal ) # Send data ( Publish to topic bumper_action/goal )
        action_client.wait_for_result() # wait for result
        
        result = action_client.get_result()
        if result.bumper_hit: 
            rospy.loginfo( 'bumper hit!' )
            return 'succeeded'
            
        else:
            rospy.loginfo( 'faild!' )
            #self.counter += 1
            if self.counter < 3:
                self.counter += 1
                return 'failed'
            else:
                return 'aborted'
                #return 'failed'

        

#class State2(smach.State):
#    def __init__(self):
#        smach.State.__init__(self, outcomes=['done'])
#
#    def execute(self, userdata):
#        rospy.loginfo('Executing state STATE2')
#        rospy.sleep(2.0)
#        return 'done'

def main():
    rospy.init_node('A_test_state_machine')
   
    # Create the top level SMACH state machine
    sm_top = smach.StateMachine(outcomes=['done', 'exit'])
    
    # Open the container
    with sm_top:
        goal1=MoveBaseGoal()
        goal1.target_pose.header.frame_id = "dtw_robot1/map"
        goal1.target_pose.pose.position.x = 2.5
        goal1.target_pose.pose.position.y = 2.8
        goal1.target_pose.pose.orientation.w = 1.0
        smach.StateMachine.add('MOVE1', smach_ros.SimpleActionState('/dtw_robot1/move_base', MoveBaseAction, goal=goal1), transitions={'succeeded':'YOLO_START', 'preempted':'MOVE1', 'aborted':'MOVE1'})

        
        sm_sub = smach.StateMachine(outcomes=['done'])
        with sm_sub:


            smach.StateMachine.add('YOLO', Callback(), transitions={'done':'done'})
        smach.StateMachine.add('YOLO_START', sm_sub, transitions={'done':'MOVE2'}) 

        goal2=MoveBaseGoal()
        goal2.target_pose.header.frame_id = "dtw_robot1/map"
        goal2.target_pose.pose.position.x = 4.0
        goal2.target_pose.pose.position.y = 3.8
        goal2.target_pose.pose.orientation.w = 1.0
        smach.StateMachine.add('MOVE2', smach_ros.SimpleActionState('/dtw_robot1/move_base', MoveBaseAction, goal=goal2), transitions={'succeeded':'MOVE3', 'preempted':'exit', 'aborted':'exit'})

        goal=MoveBaseGoal()
        goal.target_pose.header.frame_id = "dtw_robot1/map"
        goal.target_pose.pose.position.x = 6.8
        goal.target_pose.pose.position.y = 1.9
        goal.target_pose.pose.orientation.w = 1.0
        smach.StateMachine.add('MOVE3', smach_ros.SimpleActionState('/dtw_robot1/move_base', MoveBaseAction, goal=goal), transitions={'succeeded':'DETECT_AREA', 'preempted':'exit', 'aborted':'MOVE2'})
         
        sm_sub = smach.StateMachine(outcomes=['done','exit'])
        with sm_sub:


            smach.StateMachine.add('Detect_Start', State1(), transitions={'succeeded':'done', 'failed':'exit', 'aborted':'done'})
            #smach.StateMachine.add('Retry', State2(), transitions={'done':'Detect_Start'})
        smach.StateMachine.add('DETECT_AREA', sm_sub, transitions={'done':'MOVE4','exit':'DETECT_AREA'}) 

        goal3=MoveBaseGoal()
        goal3.target_pose.header.frame_id = "dtw_robot1/map"
        goal3.target_pose.pose.position.x = 7.0
        goal3.target_pose.pose.position.y = 0.0
        goal3.target_pose.pose.orientation.w = 1.0
        smach.StateMachine.add('MOVE4', smach_ros.SimpleActionState('/dtw_robot1/move_base', MoveBaseAction, goal=goal3), transitions={'succeeded':'exit', 'preempted':'MOVE1', 'aborted':'MOVE2'})

# Execute SMACH plan
    sis = smach_ros.IntrospectionServer('smach_server', sm_top, '/SM_ROOT')
    sis.start()
    outcome = sm_top.execute()
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
