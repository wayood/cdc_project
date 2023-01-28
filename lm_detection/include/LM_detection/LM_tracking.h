#include <ros/ros.h>
#include <lm_detection/Position_array.h>
#include <lm_detection/Position.h>
#include <lm_detection/Bounding_Box.h>
#include <lm_detection/Bounding_Box_array.h>
#include <sensor_msgs/Image.h>
#include <vector>
#include <iostream>
#include <string>
#include <cv_bridge/cv_bridge.h>
#include <visualization_msgs/Marker.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/opencv.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

class LM_tracking{
    public:
        LM_tracking();
        ~LM_tracking();
        void process();
        void Camera_callback(const sensor_msgs::CameraInfoConstPtr&);
        void trackingcallback(const lm_detection::Bounding_Box_array&);
        void depthImageCallback(const sensor_msgs::ImageConstPtr&);
        void camera2worldframe(std::vector<int>&,std::string&);
        void tf_broadcast_and_lookuptransform(cv::Point3d&,std::string&,int);
        void LM_rviz_publish(const lm_detection::Position_array&);
        void miss_lm_found(lm_detection::Position_array&);
        std::vector<int> depth_estimate();
        lm_detection::Bounding_Box_array tracking_bounding_boxes;
        lm_detection::Position_array LM_position_array,prev_LM_position_array;
        lm_detection::Position LM_position;
        cv_bridge::CvImagePtr cv_ptr_depth;
        std::vector<int> sum_depth,prev_sum_depth;
        tf2_ros::TransformBroadcaster dynamic_br_;
        tf2_ros::Buffer tfBuffer;
        tf2_ros::Buffer tfBuffer_miss;
        bool flag_track = false;
        image_geometry::PinholeCameraModel cam_model;
        int count_id;
        int bbox_number;
    protected:
        lm_detection::Position_array miss_lm_position;

    private:
        ros::NodeHandle nh;
        ros::Subscriber sub_tracking_bbox,sub_depth;
        ros::Subscriber sub_camera_info;
        ros::Publisher pub_LM_current,marker_pub;    
};