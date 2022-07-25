#ifndef __CDC_NAVIGATION_H
#define __CDC_NAVIGATION_H

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <waypoint_generator/Waypoint.h>
#include <waypoint_generator/Waypoint_array.h>

class Navigation
{
public:
    Navigation(void);
    void process(void);
    void wp_callback(const waypoint_generator::Waypoint_array&);
    void odom_callback(const nav_msgs::OdometryConstPtr&);
    geometry_msgs::Pose getRobotCurrentPosition(void);
    void nextGoal(const waypoint_generator::Waypoint_array&,int);
protected:
    ros::NodeHandle nh;
    ros::Subscriber wp_init_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher goal_pub_;
    waypoint_generator::Waypoint wp;
    waypoint_generator::Waypoint_array wp_array_subscribe;
    nav_msgs::OdometryConstPtr odom_subscribe;
    bool wp_flag;
};

#endif