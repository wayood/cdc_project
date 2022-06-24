#!/usr/bin/env python

import rospy
import csv
from visualization_msgs.msg import Marker
from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from nav_msgs.msg import Path

rospy.init_node("waypoint_manager")
path_pub = rospy.Publisher("/path", Path, queue_size=50)
rate = rospy.Rate(25)
io=sum([1 for _ in open('path_data.csv')])
while not rospy.is_shutdown():
      with open('./path_data.csv') as f:
         counter = 0
         reader = csv.reader(f)
         # header = next(reader)
         path_header = Header()
         path_header.seq = 0
         path_header.stamp = rospy.Time.now()
         path_header.frame_id = "map"
         path = Path()
         path.header = path_header         
         line = [row for row in reader]
         poses_list = []
         for i in range(1,io):
            marker_data = PoseStamped()
            marker_data.header = path_header
            marker_data.header.seq = counter
            marker_data.pose.position.x = float(line[i][1])
            marker_data.pose.position.y = float(line[i][2])
            marker_data.pose.position.z = float(line[i][3])

            marker_data.pose.orientation.x= float(line[i][4])
            marker_data.pose.orientation.y= float(line[i][5])
            marker_data.pose.orientation.z= float(line[i][6])
            marker_data.pose.orientation.w= float(line[i][7])

            poses_list.append(marker_data)
            counter +=1
         path.poses =poses_list
         path_pub.publish(path)

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

        
