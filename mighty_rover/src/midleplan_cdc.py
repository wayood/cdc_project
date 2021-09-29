#!/usr/bin/env python
# import roslib; roslib.load_manifest('mighty_rover')
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

def main():
        rospy.init_node('midleplan_cdc', anonymous=True)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped)
        rospy.Subscriber('/move_base/NavfnROS/plan',Path,callback_con)
        rospy.spin()

def callback_con(msg):
        # print(len(msg.poses))
        # print(msg.poses[len(msg.poses)-1].pose.position.x)
        r = rospy.Rate(10)  # 10hz
        path_header = msg.poses[0].header
        path_header.seq = 0
        path_header.stamp = rospy.Time.now()
        path_header.frame_id = "map"
        path = Path()
        path.header = path_header
        pose=cdc_path(msg.poses)
        path_pub = rospy.Publisher("/move_base/NavfnROS/plan", Path, queue_size=10)
        path.poses=pose
        print(path)
        path_pub.publish(path)
        r.sleep()    

        
  
def cdc_path(pose):
        poses = []
        for i in range(len(pose)):
          path=PoseStamped()
          path.pose.position.x = pose[i].pose.position.x 
          path.pose.position.y = pose[i].pose.position.y 
          path.pose.position.z = pose[i].pose.position.z 
          path.pose.orientation.x = pose[i].pose.orientation.x
          path.pose.orientation.y = pose[i].pose.orientation.y
          path.pose.orientation.z = pose[i].pose.orientation.z
          path.pose.orientation.w = pose[i].pose.orientation.w
          path.header = pose[i].header
          path.header.seq = i
          poses.append(path)
        return poses

if __name__ == '__main__':
   
   try:
       while not rospy.is_shutdown():
          main()

   except ZeroDivisionError:
       print('Error')
