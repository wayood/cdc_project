#!/usr/bin/env python

import rospy
import smach
import smach_ros
import os
from visualization_msgs.msg import Marker, MarkerArray
import tf
import tf2_ros
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Quaternion, PointStamped, PoseWithCovarianceStamped, TransformStamped, PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
import math
#import RPi.GPIO as GPIO


class set_route(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

    def execute(self, userdata):

        global route_name

        print("Please input route name...")
        route_name = raw_input("route name: ")

        return 'done'
        

class target_navigation(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
        self.test_route_array = []
        self.target_count = 1

        self.marker_publisher = rospy.Publisher('set_marker', MarkerArray, queue_size=10)

        self.flag_continue_navigation = True
        self.test_count = 0

        self.ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.rate = rospy.Rate(10.0)
        self.br = tf2_ros.StaticTransformBroadcaster()


    def execute(self, userdata):

        global route_name

        path = '/home/klab/catkin_ws/src/ros_beginner/src/route/' + route_name + '/target' + str(self.target_count) + '.txt'

        if os.path.exists(path):
            pass
        else:
            self.target_count += 1
            return 'done'
        
        with open(path) as test_read:
            while True:
                test_route = test_read.readline().split()
                test_route_float = [float(i) for i in test_route]
                self.test_route_array.append(test_route_float)
                if not test_route:
                    break
        self.test_route_array.pop() # delete last element [] in array

        print("Start target navigation!!")

        for i in (range(len(self.test_route_array))):
            markerArray = MarkerArray()
            marker = Marker()
            marker.header.frame_id = "map"
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = self.test_route_array[i][0]
            marker.pose.position.y = self.test_route_array[i][1]
            marker.pose.position.z = 0.0
            marker.id = i + 1000
            markerArray.markers.append(marker)

            marker = Marker()
            marker.header.frame_id = "map"
            marker.type = Marker.TEXT_VIEW_FACING
            marker.text = "target" + str(i+1)
            marker.action = marker.ADD
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = self.test_route_array[i][0] + 0.1
            marker.pose.position.y = self.test_route_array[i][1]
            marker.pose.position.z = 0.0
            marker.id = i
            markerArray.markers.append(marker)
            self.marker_publisher.publish(markerArray)
            i += 1


        for i in (range(len(self.test_route_array))):
            print("route No." + str(self.target_count))
            print("target No." + str(i+1) +": ")
            print(self.test_route_array[i])

            # set msg here
            transformstamped = TransformStamped()
            transformstamped.header.stamp = rospy.Time.now()
            transformstamped.header.frame_id = "map"
            transformstamped.child_frame_id = "target"
            transformstamped.transform.translation.x = self.test_route_array[i][0]
            transformstamped.transform.translation.y = self.test_route_array[i][1]
            transformstamped.transform.translation.z = self.test_route_array[i][2]
            transformstamped.transform.rotation.x    = self.test_route_array[i][3]
            transformstamped.transform.rotation.y    = self.test_route_array[i][4]
            transformstamped.transform.rotation.z    = self.test_route_array[i][5]
            transformstamped.transform.rotation.w    = self.test_route_array[i][6]
            self.br.sendTransform(transformstamped)

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x    =  self.test_route_array[i][0]
            goal.target_pose.pose.position.y    =  self.test_route_array[i][1]
            goal.target_pose.pose.position.z    =  self.test_route_array[i][2]
            goal.target_pose.pose.orientation.x =  self.test_route_array[i][3]
            goal.target_pose.pose.orientation.y =  self.test_route_array[i][4]
            goal.target_pose.pose.orientation.z =  self.test_route_array[i][5]
            goal.target_pose.pose.orientation.w =  self.test_route_array[i][6]
   
            self.ac.send_goal(goal)

            rospy.sleep(1) ### waiting transform of target marker
            
            print_count = 0
            while self.flag_continue_navigation:

                t = self.tfBuffer.lookup_transform('target', 'base_footprint', rospy.Time())
                distance_from_robot_to_target = [t.transform.translation.x, t.transform.translation.y]

                distance_difference = math.sqrt(distance_from_robot_to_target[0] ** 2 + distance_from_robot_to_target[1] ** 2)

                if(print_count == 0):
                    rospy.loginfo("Going to target...")
                    print_count = 1

                if(distance_difference < 1.5):
                    rospy.loginfo("Next target...")
                    self.flag_continue_navigation = False


            self.flag_continue_navigation = True

        self.target_count += 1
        del self.test_route_array[:]

        return 'done'



class goal_navigation(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'next'])
        self.test_route_array = []
        self.goal_count = 1

        self.goalmarker_id = 1
        self.goalmarker_publisher = rospy.Publisher('set_goalmarker', MarkerArray, queue_size=10)

        self.flag_continue_navigation = True
        self.test_count = 0

        self.distance_from_robot_to_target = []
        self.ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.br = tf2_ros.StaticTransformBroadcaster()

        while not self.ac.wait_for_server(rospy.Duration(5)):
            rospy.loginfo("Waiting for the move_base action server to come up")
        rospy.loginfo("The server comes up")
        print("Please set first route goal...")



    def execute(self, userdata):

        global route_name

        path = '/home/robodev/catkin_ws_tutorial_turtlebot3/src/navigation/script/route/' + route_name + '/goal' + str(self.goal_count) + '.txt'
        
        with open(path) as test_read:
            test_route = test_read.read().split()
            self.test_route_array = [float(i) for i in test_route]

        print("Start goal navigation!!")
        print("route No." + str(self.goal_count))
        print("goal: ")
        print(self.test_route_array)

        
        # set msg here
        transformstamped = TransformStamped()
        transformstamped.header.stamp = rospy.Time.now()
        transformstamped.header.frame_id = "map"
        transformstamped.child_frame_id = "goal"
        transformstamped.transform.translation.x = self.test_route_array[0]
        transformstamped.transform.translation.y = self.test_route_array[1]
        transformstamped.transform.translation.z = self.test_route_array[2]
        transformstamped.transform.rotation.x    = self.test_route_array[3]
        transformstamped.transform.rotation.y    = self.test_route_array[4]
        transformstamped.transform.rotation.z    = self.test_route_array[5]
        transformstamped.transform.rotation.w    = self.test_route_array[6]
        self.br.sendTransform(transformstamped)

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id    = 'map'
        goal.target_pose.header.stamp       = rospy.Time.now()
        goal.target_pose.pose.position.x    = self.test_route_array[0]
        goal.target_pose.pose.position.y    = self.test_route_array[1]
        goal.target_pose.pose.position.z    = self.test_route_array[2]
        goal.target_pose.pose.orientation.x = self.test_route_array[3]
        goal.target_pose.pose.orientation.y = self.test_route_array[4]
        goal.target_pose.pose.orientation.z = self.test_route_array[5]
        goal.target_pose.pose.orientation.w = self.test_route_array[6]

        self.ac.send_goal(goal)

        rospy.sleep(1) ### waiting transform of target marker


        while self.flag_continue_navigation:
            succeeded = self.ac.wait_for_result(rospy.Duration(120))
            state = self.ac.get_state()

            if succeeded:
                rospy.loginfo(state)
                self.flag_continue_navigation = False

            else:
                rospy.loginfo(state)
        
        self.test_count = 0
        self.flag_continue_navigation = True
        del self.test_route_array[:]

        self.goal_count += 1
        path = '/home/robodev/catkin_ws_tutorial_turtlebot3/src/navigation/script/route/' + route_name + '/goal' + str(self.goal_count) + '.txt'

        if os.path.exists(path):
            return 'next'
        else:
            print("Finish navgation!!")
            return 'done'

# main
def main():
    rospy.init_node('navigation')

    sm_top = smach.StateMachine(outcomes=['succeeded'])
    with sm_top:
        smach.StateMachine.add('set_route', set_route(), transitions={'done':'target_navigation'})
        smach.StateMachine.add('target_navigation', target_navigation(), transitions={'done':'goal_navigation'})
        smach.StateMachine.add('goal_navigation', goal_navigation(), transitions={'next':'target_navigation', 'done':'succeeded'})

    sis = smach_ros.IntrospectionServer('smach_server', sm_top, '/navigation')
    sis.start()
    outcome = sm_top.execute()
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()