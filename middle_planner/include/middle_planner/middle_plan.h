#ifndef __CDC_PLANNER_H
#define __CDC_PLANNER_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>
#include <Eigen/Dense>
#include <math.h>
#include <vector>
#include <algorithm>

class CDCPlanner
{
public:
    CDCPlanner(void);
    void process(void);
    void path_callback(const nav_msgs::Path&);
    void lm_x_callback(const std_msgs::Float32MultiArray::ConstPtr&);
    void lm_y_callback(const std_msgs::Float32MultiArray::ConstPtr&);
    void prepare(Eigen::MatrixXf&);
    Eigen::MatrixXf a_mat(Eigen::MatrixXf&,Eigen::MatrixXf&);
    Eigen::MatrixXf CDC_plan(const nav_msgs::Path&,Eigen::MatrixXf&);
    void CDC_publish(const nav_msgs::Path&,Eigen::MatrixXf&);
protected:
    ros::NodeHandle nh;
    ros::Subscriber path_sub;
    ros::Subscriber lm_x_sub;
    ros::Subscriber lm_y_sub;
    ros::Publisher path_pub;
    int num;
    float lm_y[1000];
    float lm_x[1000];
    nav_msgs::Path path;
    nav_msgs::Path middle_path;
    geometry_msgs::PoseStamped middle_pose;
    bool path_subscribed;
    bool lm_x_subscribed;
    bool lm_y_subscribed;
};

#endif