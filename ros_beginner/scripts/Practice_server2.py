#!/usr/bin/env python
import rospy
import actionlib
from geometry_msgs.msg import Twist
#from kobuki_msgs.msg import BumperEvent
from ros_beginner.msg import Practice2Action
from ros_beginner.msg import Practice2Result
from ros_beginner.msg import Practice2Feedback

from sync_sub.msg import some_position
from sync_sub.msg import some_position2


import math
from darknet_ros_msgs.msg import BoundingBoxes

a=0
class Action(object):
    def __init__(self):
        self._vel_pub = rospy.Publisher( '/dtw_robot1/diff_drive_controller/cmd_vel', Twist, queue_size = 10 )
        #self._max_vel = rospy.get_param( '~max_vel', 0.5 )

        ## exuecute_cb: Server to actually run
        ## auto_start: auto start ( false )
        self._action_server = actionlib.SimpleActionServer( 'action', Practice2Action, execute_cb = self.go_until_bumper, auto_start = False )
        self._hit_bumper = False
        self._action_server.start()
        self._sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes ,self.callback)
        self._vel = Twist()
        self._red_area = 0

    def callback(self, data):
        
        self._hit_bumper = False                      #TO USE REPETITION

        for self._box in data.bounding_boxes:
            #rospy.loginfo(self._box.Class)
            a=0


    def go_until_bumper(self, goal): # Action's server
        
        #print( goal.target_vel )
        r = rospy.Rate( 10.0 ) # 10kHz
        zero_vel = Twist()
        
        # 1 second in 10 loops
        for i in range( 10 * goal.timeout_sec ): # timeout_sec is input by commands
            if self._action_server.is_preempt_requested(): # stop intruction
                self._action_server.set_preempted() # Send that action was blocked by user request
                break

            if self._hit_bumper:
                #self._vel_pub.publish( zero_vel )
                #rospy.loginfo(zero_vel)

                self._vel.linear.x = 0
                self._vel_pub.publish(self._vel)
                rospy.loginfo(self._vel)
                break
            
            else:
                #rospy.loginfo(box.xmax)
                Velocity=(((4.00-(((self._box.xmin + self._box.xmax)*0.01)/2))/2)*0.25*math.pi)
                if self._box.Class == "bottle":
                    #rospy.loginfo("found_angular!!")
                    self._vel.angular.z = Velocity
                    self._vel_pub.publish(self._vel)            #2
                else:
                    self._vel.angular.z = 0.1
                    self._vel_pub.publish(self._vel)    #####----ADD----####

                Width=(self._box.xmax-self._box.xmin)
                if self._box.Class == "bottle":
                    if Width < 200:
                        #rospy.loginfo("forward!!")
                        self._vel.linear.x = 0.1
                        self._vel_pub.publish(self._vel)
                    elif 200 < Width < 220:
                        #rospy.loginfo("slow_forward!!")
                        self._vel.linear.x = 0.03
                        self._vel_pub.publish(self._vel)
                    elif Width >230:
                            #rospy.loginfo("backward!!")
                            self._vel.linear.x = -0.03
                            self._vel_pub.publish(self._vel)
                            #rospy.loginfo('tracking end!!!!')
                            self._hit_bumper = True
                            break
                
                #feedback = Practice2Feedback( current_vel = goal.target_vel )
                #self._action_server.publish_feedback( feedback ) # Pass an instance of GoUntilBumperFeedback class to publish_feedback()
                
                
                #feedback = Practice2Feedback( sample = 'ADJUST' )
                #self._action_server.publish_feedback( feedback )
            r.sleep()


        # Call set_succeeded() by inserting the result of bumper_hit into an instance of class GoUntilBumperResult
        result = Practice2Result( bumper_hit = self._hit_bumper )
        self._action_server.set_succeeded( result )

        
if __name__ == '__main__':
    rospy.init_node( 'action' )
    action = Action()
    rospy.spin()