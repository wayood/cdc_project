#!/usr/bin/env python

import rospy
import csv
from visualization_msgs.msg import Marker
from move_base_msgs.msg import MoveBaseActionGoal

def callback(data):
    pos = data.goal.target_pose.pose
    print "{0},{1},0.0,0.0,0.0,{2},{3},".format(pos.position.x,pos.position.y,pos.orientation.z,pos.orientation.w)


rospy.init_node("waypoint_manager")

pub = rospy.Publisher("waypoint", Marker, queue_size = 10)
rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, callback)

rate = rospy.Rate(25)


while not rospy.is_shutdown():
    with open('/root/catkin_ws/src/ros_beginner/src/wp/wp2.csv', 'r') as f:
        counter = 0
        reader = csv.reader(f)
        #header = next(reader)

        for row in reader:
            # Mark arrow
            marker_data = Marker()
            marker_data.header.frame_id = "map"
            marker_data.header.stamp = rospy.Time.now()

            marker_data.ns = "basic_shapes"
            marker_data.id = counter

            marker_data.action = Marker.ADD

            marker_data.pose.position.x = map(float,row)[1]
            marker_data.pose.position.y = map(float,row)[2]
            marker_data.pose.position.z = map(float,row)[3]

            marker_data.pose.orientation.x=map(float,row)[4]
            marker_data.pose.orientation.y=map(float,row)[5]
            marker_data.pose.orientation.z=map(float,row)[6]
            marker_data.pose.orientation.w=map(float,row)[7]

            marker_data.color.r = 1.0
            marker_data.color.g = 0.0
            marker_data.color.b = 0.0
            marker_data.color.a = 1.0
            marker_data.scale.x = 1
            marker_data.scale.y = 0.2
            marker_data.scale.z = 0.2

            marker_data.lifetime = rospy.Duration()

            marker_data.type = 0

            pub.publish(marker_data)
            counter +=1


            # Mark num
            marker_data = Marker()
            marker_data.header.frame_id = "map"
            marker_data.header.stamp = rospy.Time.now()

            marker_data.ns = "basic_shapes"
            marker_data.id = counter

            marker_data.action = Marker.ADD

            marker_data.pose.position.x = map(float,row)[1]
            marker_data.pose.position.y = map(float,row)[2]
            marker_data.pose.position.z = map(float,row)[3]

            marker_data.pose.orientation.x=map(float,row)[4]
            marker_data.pose.orientation.y=map(float,row)[5]
            marker_data.pose.orientation.z=map(float,row)[6]
            marker_data.pose.orientation.w=map(float,row)[7]

            marker_data.color.r = 0.0
            marker_data.color.g = 0.0
            marker_data.color.b = 0.0
            marker_data.color.a = 1.0
            marker_data.scale.x = 0.8
            marker_data.scale.y = 0.8
            marker_data.scale.z = 1.2 #0.8

            marker_data.lifetime = rospy.Duration()

            marker_data.type = Marker.TEXT_VIEW_FACING
            marker_data.text = str(int(map(float,row)[0]))

            pub.publish(marker_data)
            counter +=1

    rate.sleep()

rospy.spin()