#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <iostream>
#include <message_filters/time_synchronizer.h>
#include <std_msgs/Header.h>
#include <vector>
#include <image_geometry/pinhole_camera_model.h>
#include <string>
#include <message_filters/subscriber.h>
#include <lm_detection/Position.h>
#include <lm_detection/Position_array.h>
#include <lm_detection/Bounding_Box.h>
#include <lm_detection/Bounding_Box_array.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <waypoint_generator/Waypoint.h>
#include <waypoint_generator/Waypoint_array.h>
#include <math.h>
#include <sstream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/LU>

class depth_estimater{
public:
    depth_estimater();
    ~depth_estimater();
    void rgbImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void depthImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void camera_info_callback(const sensor_msgs::ImageConstPtr& rgb_msg,const sensor_msgs::ImageConstPtr& depth_msg,const sensor_msgs::CameraInfoConstPtr& cam_info);
    void main();
    void LM_rviz_publish(const lm_detection::Position&,int);
    void wp_callback(const waypoint_generator::Waypoint_array&);;
    cv::Point2d LM_position_raw_data;
    tf2_ros::TransformBroadcaster dynamic_br_;
    tf2_ros::Buffer tfBuffer;
    std::vector<std::vector <double>> LM_point;
    std::vector<std::vector <int>> BBox_rectangle;
    lm_detection::Position_array LM_position_array;
    lm_detection::Position LM_position;
    lm_detection::Bounding_Box bbox;
    lm_detection::Bounding_Box_array bbox_array;
    cv_bridge::CvImagePtr cv_ptr_depth;
    image_geometry::PinholeCameraModel cam_model;
    std::vector<int> sum_depth;
    bool flag_depth = false;
    bool flag_bbox = false;
    bool camera_info_flag = false;
    bool wp_flag;
    ros::Publisher LM_pub;
    // tf::TransformListener listener;
private:
    ros::NodeHandle nh;
    ros::Subscriber sub_rgb, sub_depth,wp_init_sub_;
    ros::Publisher marker_pub,bbox_pub;
    
};