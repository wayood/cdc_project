#ifndef GO_FORWARD_RECOVERY_H_
#define GO_FORWARD_RECOVERY_H_
#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
// #include <tf/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <ros/ros.h>
#include <base_local_planner/costmap_model.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <angles/angles.h>
#include <sensor_msgs/LaserScan.h>

#define RANGE_MAX 5.6

namespace go_forward_recovery{
  class GoForwardRecovery : public nav_core::RecoveryBehavior {
    public:
      GoForwardRecovery();

      virtual void initialize(std::string name,  tf2_ros::Buffer*tf, 
          costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap);

      virtual void runBehavior();

      ~GoForwardRecovery();

    private:
      costmap_2d::Costmap2DROS* global_costmap_, *local_costmap_;
      costmap_2d::Costmap2D costmap_;
      std::string name_;
      ros::NodeHandle n;
      ros::Publisher vel_pub;
      ros::Subscriber scan_sub;
      geometry_msgs::Twist cmd_vel;
      bool initialized_;
      base_local_planner::CostmapModel* world_model_;
      double null_check(double target);
      void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
      int sub_n, sub_flag;
  };
};
#endif