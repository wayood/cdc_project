#!/usr/bin/env python
import rospy
import actionlib
from geometry_msgs.msg import Twist
#from kobuki_msgs.msg import BumperEvent
from ros_beginner.msg import PracticeAction
from ros_beginner.msg import PracticeResult
from ros_beginner.msg import PracticeFeedback

from darknet_ros_msgs.msg import BoundingBoxes

class Action(object):
    def __init__(self):
        self._pub = rospy.Publisher( '/dtw_robot1/diff_drive_controller/cmd_vel', Twist, queue_size = 10 )
                    #self._sub = rospy.Subscriber( '/mobile_base/events/bumper', BumperEvent, self.bumper_callback, queue_size = 1 )
        self._sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes ,self.callback)

        self._max_vel = rospy.get_param( '~max_vel', 0.5 )

        ## exuecute_cb: Server to actually run
        ## auto_start: auto start ( false )
        self._action_server = actionlib.SimpleActionServer( 'action', PracticeAction, execute_cb = self.go_until_bumper, auto_start = False )
        self._hit_bumper = False
        self._action_server.start()
        


    #def bumper_callback( self, bumper ): # when the bumper hits
    #    self._hit_bumper = True
    def callback(self,data):
         for box in data.bounding_boxes:
            if box.Class == "bottle":
                rospy.loginfo('BOTTLE!!!')
                self._hit_bumper = True

    def go_until_bumper(self, goal): # Action's server
        print( goal.target_vel )
        r = rospy.Rate( 10.0 ) # 10kHz
        zero_vel = Twist()
        # 1 second in 10 loops
        for i in range( 10 * goal.timeout_sec ): # timeout_sec is input by commands
            if self._action_server.is_preempt_requested(): # stop intruction
                self._action_server.set_preempted() # Send that action was blocked by user request
                break

            if self._hit_bumper:
                self._pub.publish( zero_vel )
                break
            
            else:
                if goal.target_vel.linear.x > self._max_vel:
                    goal.target_vel.linear.x = self._max_vel
                self._pub.publish( goal.target_vel ) # Publish
                
                feedback = PracticeFeedback( current_vel = goal.target_vel )

                #rospy.loginfo(PracticeFeedback.current_vel)
                #rospy.loginfo(feedback)
                self._action_server.publish_feedback( feedback ) # Pass an instance of GoUntilBumperFeedback class to publish_feedback()
            r.sleep()
        # Call set_succeeded() by inserting the result of bumper_hit into an instance of class GoUntilBumperResult
        result = PracticeResult( bumper_hit = self._hit_bumper )
        self._action_server.set_succeeded( result )
if __name__ == '__main__':
    rospy.init_node( 'action' )
    action = Action()
    rospy.spin()