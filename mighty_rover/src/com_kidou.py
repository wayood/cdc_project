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
io=sum([1 for _ in open('tf02.csv')])
while not rospy.is_shutdown():
      with open('./tf02.csv') as f:
         counter = 0
         reader = csv.reader(f)
         # header = next(reader)         
         line = [row for row in reader]
         for i in range(1,io):
          if line[i][3] == "odom" and line[i][4] == "base_link":
            marker_data = Marker()
            marker_data.header.frame_id = "map"
            marker_data.header.stamp = rospy.Time.now()

            marker_data.ns = "basic_shapes"
            marker_data.id = counter
            marker_data.action = Marker.ADD
            marker_data.pose.position.x = float(line[i][5])
            marker_data.pose.position.y = float(line[i][6])
            marker_data.pose.position.z = float(line[i][7])

            marker_data.pose.orientation.x= float(line[i][8])
            marker_data.pose.orientation.y= float(line[i][9])
            marker_data.pose.orientation.z= float(line[i][10])
            marker_data.pose.orientation.w= float(line[i][11])

            marker_data.color.r = 1.0
            marker_data.color.g = 0.0
            marker_data.color.b = 0.0
            marker_data.color.a = 1.0
            marker_data.scale.x = 0.1
            marker_data.scale.y = 0.1
            marker_data.scale.z = 0.1

            marker_data.lifetime = rospy.Duration()

            marker_data.type = 2

            pub.publish(marker_data)
            counter +=1


          # Mark num
        
          # marker_data = Marker()
          # marker_data.header.frame_id = "map"
          # marker_data.header.stamp = rospy.Time.now()

          # marker_data.ns = "basic_shapes"
          # marker_data.id = counter

          # marker_data.action = Marker.ADD
          # marker_data.pose.position.x = float(line[i][5])
          # marker_data.pose.position.y = float(line[i][6])
          # marker_data.pose.position.z = float(line[i][7])

          # marker_data.pose.orientation.x= float(line[i][8])
          # marker_data.pose.orientation.y= float(line[i][9])
          # marker_data.pose.orientation.z= float(line[i][10])
          # marker_data.pose.orientation.w= float(line[i][11])

          # marker_data.color.r = 0.0
          # marker_data.color.g = 0.0
          # marker_data.color.b = 0.0
          # marker_data.color.a = 1.0
          # marker_data.scale.x = 0.1
          # marker_data.scale.y = 0.1
          # marker_data.scale.z = 0.1

          # marker_data.lifetime = rospy.Duration()

          # marker_data.type = Marker.TEXT_VIEW_FACING
          # marker_data.text = str(i)
          # pub.publish(marker_data)
          
          
          # counter +=1  

        
      rate.sleep()

