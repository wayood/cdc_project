#ifndef __CDC_PLANNER_H
#define __CDC_PLANNER_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <iostream>
#include<vector>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Header.h>
#include <waypoint_generator/Waypoint.h>
#include <waypoint_generator/Waypoint_array.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include<lm_detection/Position.h>
#include<lm_detection/Position_array.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/LU>
#include <visualization_msgs/Marker.h>
#include <math.h>
#include <vector>
#include <algorithm>

class CDCPlanner
{
public:
    CDCPlanner(void);
    void process(void);
    void wp_callback(const waypoint_generator::Waypoint_array&);
    void lm_current_callback(const lm_detection::Position_array&);
    void lm_first_callback(const lm_detection::Position_array&);
    void prepare(void);
    Eigen::MatrixXd A_matrix(const Eigen::MatrixXd&,const Eigen::MatrixXd&,const Eigen::MatrixXd&);
    void WP_publish(Eigen::MatrixXd&);
    Eigen::MatrixXd LM_first_matching_current(const Eigen::MatrixXd&);
    Eigen::MatrixXd LM_first,LM_current,LM_first_matching,wp_first,wp_new;
    std::vector<std::string> LM_first_number,LM_current_number;
protected:
    ros::NodeHandle nh;
    ros::Publisher wp_pub;
    ros::Publisher marker_pub;
    ros::Subscriber wp_init_sub;
    ros::Subscriber lm_current_sub,lm_first_sub;
    int num;
    waypoint_generator::Waypoint wp;
    waypoint_generator::Waypoint_array wp_array_subscribe;
    waypoint_generator::Waypoint_array wp_array_publish;
    lm_detection::Position_array lm_first_array,lm_current_array;
    geometry_msgs::PoseStamped middle_pose;
    Eigen::MatrixXd LM_first_stock;
    bool wp_flag;
    bool lm_flag;
};

#endif