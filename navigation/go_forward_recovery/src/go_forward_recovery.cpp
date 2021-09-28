#include <go_forward_recovery/go_forward_recovery.h>
#include <pluginlib/class_list_macros.h>
//  tf_(NULL), 
//register this planner as a RecoveryBehavior plugin
PLUGINLIB_EXPORT_CLASS(go_forward_recovery::GoForwardRecovery, nav_core::RecoveryBehavior)

namespace go_forward_recovery {
GoForwardRecovery::GoForwardRecovery(): global_costmap_(NULL), local_costmap_(NULL), 
 initialized_(false), world_model_(NULL) {} 

void GoForwardRecovery::initialize(std::string name,tf2_ros::Buffer*tf,
    costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap){
  if(!initialized_){
    name_ = name;
    // tf_ = tf;
    global_costmap_ = global_costmap;
    local_costmap_ = local_costmap;
    world_model_ = new base_local_planner::CostmapModel(*local_costmap_->getCostmap());
    initialized_ = true;
  }else{
    ROS_ERROR("You should not call initialize twice on this object, doing nothing");
  }
}

GoForwardRecovery::~GoForwardRecovery(){
  delete world_model_;
}

double GoForwardRecovery::null_check(double target){
  if(!(target > 0)){
    target = (double)RANGE_MAX;
    //ROS_WARN("RANGE OVER");
  }
  return target;
}

//range[angle]はm単位での表記
void GoForwardRecovery::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
  double center_number = (-msg->angle_min)/msg->angle_increment;
  double center = msg->ranges[center_number];
  double left = msg->ranges[center_number+128];
  double right = msg->ranges[center_number-128];
  double back = msg->ranges[center_number-256];
  center = null_check(center);
  left = null_check(left);
  right = null_check(right);
  back = null_check(back);
  //ROS_INFO("center: [%lf], left: [%lf], right: [%lf]", center, left, right);
  //ROS_INFO("center number: [%lf]", (-msg->angle_min)/msg->angle_increment);

  if(center < 0.5){
    ROS_WARN("center warning!!");
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 1.0;
  }
  if(left < 0.4){
    ROS_WARN("left warning!!");
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = -1.0;
  }
  if(right < 0.4){
    ROS_WARN("right warning!!");
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 1.0;
  }


  if(center >= 0.5 && left >= 0.4 && right >= 0.4 ){
    cmd_vel.linear.x = 0.2;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
  }

  //ROS_INFO("x: %lf, y: %lf, z: %lf", cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
  vel_pub.publish(cmd_vel);
  if(center >= 1.0 && left >= 1.0 && right >= 1.0 && back >= 1.0){
    scan_sub.shutdown();
    sub_flag = 0;
    return;
  }
}

void GoForwardRecovery::runBehavior(){
  if(!initialized_){
    ROS_ERROR("This object must be initialized before runBehavior is called");
    return;
  }

  ROS_WARN("Go forward recovery behavior started.");
  sub_flag = 1;
  scan_sub = n.subscribe("scan", 10, &GoForwardRecovery::scanCallback, this);
  vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  while(n.ok()){
    if(sub_flag == 0){
      return;
    }
  }
}
};

