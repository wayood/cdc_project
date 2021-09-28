#!/usr/bin/env python

import rospy
import csv
from move_base_msgs.msg import MoveBaseActionGoal
from sync_sub.msg import some_position
from sync_sub.msg import some_position2


class Set(object):
    def __init__(self):

        #self.rospy.init_node('goal_sub', anonymous=True)

        rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, self.callback)
        self._pub = rospy.Publisher('set_coordinate', some_position, queue_size=10)
        self._msg = some_position()
        self._num = 0

        self._f = open('/root/catkin_ws/src/ros_beginner/src/wp/wp2.csv', 'w')
        self._writer = csv.writer(self._f, lineterminator='\n')
        self._csvlist = []
        self._array = []



    def callback(self,data):
        self.pos = data.goal.target_pose.pose
        #print "[({0},{1},0.0),(0.0,0.0,{2},{3})],".format(self.pos.position.x,self.pos.position.y,self.pos.orientation.z,self.pos.orientation.w)

        
        self._array = (self._num,self.pos.position.x,self.pos.position.y,0.0,0.0,0.0,self.pos.orientation.z,self.pos.orientation.w)

        #print "{0},{1},0.0,0.0,0.0,{2},{3}".format(self._num,self.pos.position.x,self.pos.position.y,self.pos.orientation.z,self.pos.orientation.w)

        print(self._array)

        
        #self._num +=1
        self._msg.header.stamp = rospy.Time.now()        #msg
        self._msg.setting = [self.pos.position.x,self.pos.position.y,self.pos.orientation.z,self.pos.orientation.w]
        self._pub.publish(self._msg)

            #with open('/root/catkin_ws/src/ros_beginner/src/wp/wp2.csv', 'w') as f:
            #    counter = 0
            #    writer = csv.writer(f)
            #    write.writerow(self._num,self.pos.position.x,self.pos.position.y,self.pos.orientation.z,self.pos.orientation.w)
            #    header = next(writer)


        #self._csvlist.append("{0},{1},0.0,0.0,0.0,{2},{3}".format(self._num,self.pos.position.x,self.pos.position.y,self.pos.orientation.z,self.pos.orientation.w))
        self._csvlist.append(self._array)

        #rospy.loginfo(self._csvlist[0])
        
        #self._csvlist = self._csvlist[self._num]
        
        self._writer.writerows(self._csvlist)

        self._num +=1

       
        #self._f.close()



if __name__ == '__main__':
    #listener()
    rospy.init_node( 'kyolo_set' )
    goal_sub = Set()
    rospy.spin()