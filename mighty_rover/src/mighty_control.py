#!/usr/bin/env python

import rospy
import smach
import smach_ros

from smach import CBState
from std_msg.msg import Empty
from geometry_msgs.msg import Twist

@smach.cb_interface(input_keys=['lspeed'],output_keys=[],outcomes=['finished','failed'])
    def move_cb(userdata):
        rospy.loginfo('Moveing')
        cmd_topic = rospy.publisher('/diff_drive_controller/cmd_vel',Twist,queue_size=1)
        rospy.sleep(1)
        vel = Twist()
        vel.liniear.x = user_data.lspeed
        result = cmd_topic.publish(vel)
        rospy.sleep(2)
        
        if result == None:
            return 'finished'
        else:
            return 'failed'

if __name__ == '__main__':
    main()
rospy.init_node('mighty_rover')

 # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['I_have_finished'])
    sm = user_data.lspeed = 0.5
# Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('MOVE', CBState(move_cb), 
                               {'finished':'I_have_finished', 'failed':'I_have_finished'})


# Execute SMACH plan
    outcome = sm.execute()


