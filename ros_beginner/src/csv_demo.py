#!/usr/bin/env python

import csv

# FILE OPEN
f = open('/root/catkin_ws/src/ros_beginner/src/wp/wp2.csv', 'w')
writer = csv.writer(f, lineterminator='\n')

# DATA -> LIST
csvlist = []
csvlist.append("hoge")
csvlist.append("fuga")

# WRITE
writer.writerow(csvlist)

# CLOSE
f.close()