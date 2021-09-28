#include <ros/ros.h>
#include "std_msgs/String.h"

void Callback(const std_msgs::String& msg)
{
  ROS_INFO("subscribe: %s", msg.data.c_str());
}


int main(int argc, char **argv){
 ros::init(argc, argv, "goal_sub");
 ros::NodeHandle nh_;
//  ros::Subscriber goal_sub_;
 ros::Subscriber goal_sub_ = nh_.subscribe("/move_base_simple/goal", 10,Callback);
 ros::spin();
 return 0;
}
 