#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string.h>
#include <sensor_msgs/Image.h>
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <vector>
#include <lm_detection/Bounding_Box.h>
#include <lm_detection/Bounding_Box_array.h>
#include <image_transport/image_transport.h>

class optical_flow{
public:
    optical_flow();
    void process(void);
    void rgbImageCallback(const sensor_msgs::ImageConstPtr&);
    void BboxCallback(const lm_detection::Bounding_Box_array&);
    lm_detection::Bounding_Box_array Next_Bounding_Box_output(const lm_detection::Bounding_Box_array&,cv::Mat[2]);
protected:
    ros::NodeHandle nh;
    ros::Subscriber rgb_sub,bbox_sub;
    ros::Publisher bbox_pub;
    image_transport::ImageTransport it;
    image_transport::Publisher flow_pub_image,bbox_image_pub;
    bool flag_first_frame;
    bool subscribe_rgbimage = false;
    bool subscribe_bbox = false;
    std::vector<float> error;
    std::vector<std::vector <int>> lm_prev_centerpoint,lm_next_centerpoint;
    cv_bridge::CvImagePtr rgb_image;
    lm_detection::Bounding_Box_array bbox_initial,prev_bbox,next_bbox;
    cv::Mat prev_image,prev_gray_image,next_image,next_gray_image,flow;
    cv::Mat magnitude, angle, magn_norm, flow_x, flow_y, all_flow;
    cv::Mat flow_parts[2],_hsv[3], flow_[2],hsv, hsv8, bgr;
};