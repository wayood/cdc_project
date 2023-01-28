#ifndef DOUGLAS_PEUCKER_H
#define DOUGLAS_PEUCKER_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <waypoint_generator/Waypoint.h>
#include <waypoint_generator/Waypoint_array.h>
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <cmath>
#include <utility>
#include <vector>
#include <stdexcept>

class douglas_peucker{
public:
    douglas_peucker();
    ~douglas_peucker();
    typedef std::pair<double, double> Point;
    void process();
    void path_callback(const nav_msgs::Path&);
    void RamerDouglasPeucker(const std::vector<Point>&, double, std::vector<Point>&);
    double PerpendicularDistance(const Point&, const Point&, const Point&);  
    void rviz_wp_publish();  
    std::vector<Point> Path_List,Waypoint_Out;
    bool path_subscribed = false;
    waypoint_generator::Waypoint wp;
    waypoint_generator::Waypoint_array wp_array;

private:
    ros::NodeHandle nh;
    ros::Subscriber sub_path;
    ros::Publisher pub_wp,pub_marker;
};

#endif //DOUGLAS_PEUCKER_H