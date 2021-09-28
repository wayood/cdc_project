#!/usr/bin/env python
import rospy
import actionlib
from ros_beginner.msg import PracticeAction
from ros_beginner.msg import PracticeGoal
def go_until_bumper():
    # make Action's client
    #  name: bumper_action
    #  type: GoUntilBumperAction 
    action_client = actionlib.SimpleActionClient( 'action', PracticeAction )
    action_client.wait_for_server() # Wait until the server is ready     
    # Set GoUntilBumperGaol's instance
    goal = PracticeGoal() 

    goal.target_vel.linear.x = 1.0 #0.8
    goal.timeout_sec = 10
    #rospy.loginfo(goal)
    #rospy.loginfo(goal.timeout_sec)

    action_client.send_goal( goal ) # Send data ( Publish to topic bumper_action/goal )
    action_client.wait_for_result() # wait for result
    
    result = action_client.get_result()
    #rospy.loginfo(result)
    #rospy.loginfo(result.bumper_hit)
    if result.bumper_hit: 
        rospy.loginfo( 'bumper hit!' )
        
    else:
        rospy.loginfo( 'faild!' )
if __name__ == '__main__':
    try:
        rospy.init_node( 'client' )
        go_until_bumper()


    except rospy.ROSInterruptException:
        pass