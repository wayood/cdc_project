#!/usr/bin/env python

import rospy
import actionlib
import tf
from nav_msgs.msg import Odometry
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

import csv
import actionlib




def goal_pose(pose): 
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = map(float,pose)[1]
    goal_pose.target_pose.pose.position.y = map(float,pose)[2]
    goal_pose.target_pose.pose.position.z = map(float,pose)[3]
    goal_pose.target_pose.pose.orientation.x = map(float,pose)[4]
    goal_pose.target_pose.pose.orientation.y = map(float,pose)[5]
    goal_pose.target_pose.pose.orientation.z = map(float,pose)[6]
    goal_pose.target_pose.pose.orientation.w = map(float,pose)[7]

    return goal_pose


if __name__ == '__main__':
    rospy.init_node('patrol')
    listener = tf.TransformListener()

    client = actionlib.SimpleActionClient('/move_base', MoveBaseAction) 
    client.wait_for_server()
    listener.waitForTransform("map", "/base_link", rospy.Time(), rospy.Duration(4.0))

    while True: #no limit roupe

        with open('/root/catkin_ws/src/ros_beginner/src/wp/wp2.csv', 'r') as f:
            counter = 0
            reader = csv.reader(f)
            #header = next(reader)

            for pose in reader:
                print("\nHeading to {}!".format(counter))
                goal = goal_pose(pose)
                client.send_goal(goal)
                while True:
                    now = rospy.Time.now()
                    listener.waitForTransform("map", "/base_link", now, rospy.Duration(4.0))

                    position, quaternion = listener.lookupTransform("map", "/base_link", now)

                
                    if(math.sqrt((position[0]-goal.target_pose.pose.position.x)**2 + (position[1]-goal.target_pose.pose.position.y)**2 ) <= 0.5):
                        #print "next!!"
                        #print(counter)

                        #NUM = len(pose)
                        #print(len(pose))
                        #print(pose[0])

                        #if counter == 0:
                            #print("->Reached {}! Next head to 0!".format(counter))
                        #else:
                            
                        print("->Reached {}! Next head to {}!".format(counter,counter+1))
                        counter += 1
                        break

                    else:
                        rospy.sleep(0.5)
