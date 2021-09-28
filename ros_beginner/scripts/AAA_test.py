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
        goal1.target_pose.header.frame_id = "map"
        goal1.target_pose.pose.position.x = -0.10
        goal1.target_pose.pose.position.y = 4.63
        goal1.target_pose.pose.orientation.z = 0.5
        goal1.target_pose.pose.orientation.w = 1.0
        smach.StateMachine.add('Crossroad', smach_ros.SimpleActionState('/move_base', MoveBaseAction, goal=goal1), transitions={'succeeded':'YOLO_START', 'preempted':'exit', 'aborted':'exit'})

        
        sm_sub = smach.StateMachine(outcomes=['done'])
        with sm_sub:


            smach.StateMachine.add('YOLO', Callback(), transitions={'done':'done'})
        smach.StateMachine.add('YOLO_START', sm_sub, transitions={'done':'MOVE2'}) 

        goal2=MoveBaseGoal()
        goal2.target_pose.header.frame_id = "map"
        goal2.target_pose.pose.position.x = -0.086
        goal2.target_pose.pose.position.y = 14.94
        goal1.target_pose.pose.orientation.z = 1.0
        goal2.target_pose.pose.orientation.w = 1.0
        smach.StateMachine.add('MOVE2', smach_ros.SimpleActionState('/move_base', MoveBaseAction, goal=goal2), transitions={'succeeded':'MOVE3', 'preempted':'exit', 'aborted':'exit'})

        goal3=MoveBaseGoal()
        goal3.target_pose.header.frame_id = "map"
        goal3.target_pose.pose.position.x = -7.39
        goal3.target_pose.pose.position.y = 15.68
        goal1.target_pose.pose.orientation.z = 0.5
        goal3.target_pose.pose.orientation.w = -1.0
        smach.StateMachine.add('MOVE3', smach_ros.SimpleActionState('/move_base', MoveBaseAction, goal=goal3), transitions={'succeeded':'Serach area arrival', 'preempted':'exit', 'aborted':'exit'})

        

        goal4=MoveBaseGoal()
        goal4.target_pose.header.frame_id = "map"
        goal4.target_pose.pose.position.x = -9.59 #-11.17
        goal4.target_pose.pose.position.y = 13.78 #10.246
        goal1.target_pose.pose.orientation.z = 0.5
        goal4.target_pose.pose.orientation.w = -1.0
        smach.StateMachine.add('Serach area arrival', smach_ros.SimpleActionState('/move_base', MoveBaseAction, goal=goal4), transitions={'succeeded':'DETECT_AREA', 'preempted':'exit', 'aborted':'exit'})
         
        sm_sub = smach.StateMachine(outcomes=['done','exit'])
        with sm_sub:


            smach.StateMachine.add('Detect_Start', State1(), transitions={'succeeded':'done', 'failed':'exit', 'aborted':'done'})
            #smach.StateMachine.add('Retry', State2(), transitions={'done':'Detect_Start'})
        smach.StateMachine.add('DETECT_AREA', sm_sub, transitions={'done':'MOVE5','exit':'DETECT_AREA'}) 

        goal5=MoveBaseGoal()
        goal5.target_pose.header.frame_id = "map"
        goal5.target_pose.pose.position.x = -11.17
        goal5.target_pose.pose.position.y = 10.246
        goal5.target_pose.pose.orientation.w = 1.0
        smach.StateMachine.add('MOVE5', smach_ros.SimpleActionState('/move_base', MoveBaseAction, goal=goal5), transitions={'succeeded':'MOVE6', 'preempted':'exit', 'aborted':'exit'})

        goal6=MoveBaseGoal()
        goal6.target_pose.header.frame_id = "map"
        goal6.target_pose.pose.position.x = -6.33 #-6.753
        goal6.target_pose.pose.position.y = 9.578 #9.97
        goal6.target_pose.pose.orientation.w = 1.0
        smach.StateMachine.add('MOVE6', smach_ros.SimpleActionState('/move_base', MoveBaseAction, goal=goal6), transitions={'succeeded':'Crossroad2', 'preempted':'exit', 'aborted':'exit'})

        goal7=MoveBaseGoal()
        goal7.target_pose.header.frame_id = "map"
        goal7.target_pose.pose.position.x = -5.468
        goal7.target_pose.pose.position.y = 9.199
        goal7.target_pose.pose.orientation.w = 1.0
        smach.StateMachine.add('Crossroad2', smach_ros.SimpleActionState('/move_base', MoveBaseAction, goal=goal7), transitions={'succeeded':'MOVE8', 'preempted':'exit', 'aborted':'exit'})

        goal8=MoveBaseGoal()
        goal8.target_pose.header.frame_id = "map"
        goal8.target_pose.pose.position.x = -5.358
        goal8.target_pose.pose.position.y = 1.868
        goal8.target_pose.pose.orientation.w = 1.0
        smach.StateMachine.add('MOVE8', smach_ros.SimpleActionState('/move_base', MoveBaseAction, goal=goal8), transitions={'succeeded':'exit', 'preempted':'exit', 'aborted':'exit'})

# Execute SMACH plan
    sis = smach_ros.IntrospectionServer('smach_server', sm_top, '/SM_ROOT')
    sis.start()
    outcome = sm_top.execute()
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
