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
        # i=0
        # while True:
        #   print(msg.poses[i].header.stamp)
        #   if msg.poses[i+1].header.stamp is None:
        #     break
        #   i=i+1
        # print(i)
        r = rospy.Rate(50)  # 10hz
        path_header = msg.poses[0].header
        path_header.seq = rospy.Time.now()
        path_header.stamp = 0
        path_header.frame_id = "map"
        path = Path()
        path.header = path_header
        pose=cdc_path(msg.poses)
        path_pub = rospy.Publisher("/path", Path, queue_size=50)
        path.poses=pose
        path_pub.publish(path)
        r.sleep()    

        
  
def cdc_path(pose):
    #   if i in range(len(pose.))
        path=PoseStamped()
        path.pose.position.x = pose[0].pose.position.x
        path.pose.position.y = pose[0].pose.position.y
        path.pose.position.z = pose[0].pose.position.z 
        path.pose.orientation.x = pose[0].pose.orientation.x
        path.pose.orientation.y = pose[0].pose.orientation.y
        path.pose.orientation.z = pose[0].pose.orientation.z
        path.pose.orientation.w = pose[0].pose.orientation.w
        return path

if __name__ == '__main__':
   
   try:
       while not rospy.is_shutdown():
          main()

   except ZeroDivisionError:
       print('Error')
