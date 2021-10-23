#!/usr/bin/env python
# import roslib; roslib.load_manifest('mighty_rover')
import rospy
import tf2_ros
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray
i=0
class cdc:

    def __init__(self):
        self.plan = 0
        rospy.Subscriber("/move_base_simple/goal", PoseStamped)
        rospy.Subscriber('/nav_path',Path,self.callback_con)        
  
    def main(self):
        
        
        # rospy.Subscriber('/lm_position',PoseStamped,self.callback_lm)

    def callback_con(self,msg1):
        self.plan = msg1
        
    
    # print(len(msg.poses))
    # print(msg.poses[len(msg.poses)-1].pose.position.x)
    # def middle_path(self):

    #     r = rospy.Rate(10)
    #     path_header = self.plan.poses[0].header
    #     path_header.seq = 0
    #     path_header.stamp = rospy.Time.now()
    #     path_header.frame_id = "map"
    #     path = Path()
    #     path.header = path_header
    #     self.pose = cdc_path(self.plan.poses)
    #     path_pub = rospy.Publisher("/middle_path", Path, queue_size=10)
    #     path.poses = self.pose
    #     print(path)
    #     path_pub.publish(path)
    #     r.sleep()
           
    # def callback_lm(self,msg2):
    #     global i
    #     self.lm = msg2
    #     print(i)
    #     i = i+1
    #     return self.lm
    
  
    # def cdc_path(pose):

    #     poses = []
    #     for i in range(len(pose)):
    #         path=PoseStamped()
    #         path.pose.position.x = pose[i].pose.position.x 
    #         path.pose.position.y = pose[i].pose.position.y 
    #         path.pose.position.z = pose[i].pose.position.z 
    #         path.pose.orientation.x = pose[i].pose.orientation.x
    #         path.pose.orientation.y = pose[i].pose.orientation.y
    #         path.pose.orientation.z = pose[i].pose.orientation.z
    #         path.pose.orientation.w = pose[i].pose.orientation.w
    #         path.header = pose[i].header
    #         path.header.seq = i
    #         poses.append(path)
    #     return poses



if __name__ == '__main__':
   rospy.init_node('midleplan_cdc', anonymous=True)

   rospy.sleep(3.0)
   middle = cdc()
   middle.main()
   while not rospy.is_shutdown():
       rospy.sleep(0.1)

   
