#!/usr/bin/env python

import rospy
import actionlib
import tf
from nav_msgs.msg import Odometry
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal



#waypoints = [[(2.73717689514,2.83390903473,0.0),(0.0,0.0,0.434587749815,0.900629495248)],
#             [(4.3242316246,3.8101131916,0.0),(0.0,0.0,-0.424790763071,0.905291559449)],
#             [(4.55712509155,1.72027885914,0.0),(0.0,0.0,-0.724308299691,0.689476241069)],
#             [(5.70781898499,1.43851518631,0.0),(0.0,0.0,0.0131690411475,0.999913284418)],
#             [(6.67099618912,2.33934783936,0.0),(0.0,0.0,0.67851462394,0.734586894179)],
#             [(6.76753616333,3.57610726357,0.0),(0.0,0.0,0.696893645393,0.717174488539)],
#             [(6.89842224121,0.835528492928,0.0),(0.0,0.0,-0.694362082408,0.719625804508)]
#            ]

waypoints = [[(0.0498600006104,-0.0486715696752,0.0),(0.0,0.0,0.00852758159962,0.999963639515)],
             [(1.6756477356,1.70367527008,0.0),(0.0,0.0,0.706691106633,0.707522211529)],
             [(1.55858612061,11.284986496,0.0),(0.0,0.0,0.929357456141,0.369181146209)],
             [(-2.25628852844,15.2056665421,0.0),(0.0,0.0,0.703903758767,0.710295359969)],
             [(-2.13903999329,25.3130474091,0.0),(0.0,0.0,0.69454954432,0.719444876613)],
             [(-0.0507526397705,25.5190486908,0.0),(0.0,0.0,-0.0330458533039,0.999453836643)],
             [(6.20515632629,25.1697330475,0.0),(0.0,0.0,-0.00100615638561,0.999999493825)],
             [(24.4176158905,25.2156944275,0.0),(0.0,0.0,-0.0235719601921,0.999722142744)],
             [(24.5409927368,30.7279186249,0.0),(0.0,0.0,0.692115449249,0.721786814032)],
             [(32.5561256409,30.6371517181,0.0),(0.0,0.0,0.0102172876831,0.999947802154)]
            ]


def goal_pose(pose): 
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'dtw_robot1/map'
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = pose[0][2]
    goal_pose.target_pose.pose.orientation.x = pose[1][0]
    goal_pose.target_pose.pose.orientation.y = pose[1][1]
    goal_pose.target_pose.pose.orientation.z = pose[1][2]
    goal_pose.target_pose.pose.orientation.w = pose[1][3]

    return goal_pose


if __name__ == '__main__':
    rospy.init_node('patrol')
    listener = tf.TransformListener()

    client = actionlib.SimpleActionClient('dtw_robot1/move_base', MoveBaseAction) 
    client.wait_for_server()
    listener.waitForTransform("dtw_robot1/map", "dtw_robot1/base_link", rospy.Time(), rospy.Duration(4.0))

    while True:
        for pose in waypoints: 
            goal = goal_pose(pose)
            client.send_goal(goal)
            while True:
                now = rospy.Time.now()
                listener.waitForTransform("dtw_robot1/map", "dtw_robot1/base_link", now, rospy.Duration(4.0))

                position, quaternion = listener.lookupTransform("dtw_robot1/map", "dtw_robot1/base_link", now)

               
                if(math.sqrt((position[0]-goal.target_pose.pose.position.x)**2 + (position[1]-goal.target_pose.pose.position.y)**2 ) <= 1):
                    print "next!!"
                    break

                else:
                    rospy.sleep(0.5)
