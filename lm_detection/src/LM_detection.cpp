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
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
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
    void LM_rviz_publish(const cv::Point3d&,int);
    cv::Point2d LM_position_raw_data;
    tf2_ros::TransformBroadcaster dynamic_br_;
    tf2_ros::Buffer tfBuffer;
    std::vector<std::vector <double>> LM_point;
    std::vector<std::vector <int>> BBox_rectangle;
    lm_detection::Position_array LM_position_array;
    lm_detection::Position LM_position;
    cv_bridge::CvImagePtr cv_ptr_depth;
    image_geometry::PinholeCameraModel cam_model;
    std::vector<int> sum_depth;
    bool flag_depth = false;
    bool flag_bbox = false;
    bool camera_info_flag = false;
    ros::Publisher LM_pub;
    // tf::TransformListener listener;
private:
    ros::NodeHandle nh;
    ros::Subscriber sub_rgb, sub_depth;
    ros::Publisher marker_pub;
    
};
 
depth_estimater::depth_estimater(){
    sub_rgb = nh.subscribe<sensor_msgs::Image>("/saliency/image", 5, &depth_estimater::rgbImageCallback, this);
    sub_depth = nh.subscribe<sensor_msgs::Image>("/camera/depth/image_raw", 5, &depth_estimater::depthImageCallback, this);
    LM_pub = nh.advertise<lm_detection::Position_array>("lm_position",1);
    marker_pub = nh.advertise<visualization_msgs::Marker>("lm_position_marker", 1);
}
 
depth_estimater::~depth_estimater(){
}
void depth_estimater::camera_info_callback(const sensor_msgs::ImageConstPtr& rgb_msg,const sensor_msgs::ImageConstPtr& depth_msg,const sensor_msgs::CameraInfoConstPtr& cam_info){
    std::cout << "Sucsess !!" << std::endl;
    camera_info_flag = true;
    cam_model.fromCameraInfo(cam_info);
}
void depth_estimater::rgbImageCallback(const sensor_msgs::ImageConstPtr& msg){
 

    cv_bridge::CvImagePtr cv_ptr;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<std::vector<cv::Point>> approxes;
    std::vector<cv::Rect> bboxes;    

    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    }catch (cv_bridge::Exception& ex){
        ROS_ERROR("error");
        exit(-1);
    }
    cv::Mat normImage;
    const double threshold = 150.0;
    const double maxValue = 255.0;
    
    cv::threshold(cv_ptr->image, normImage, threshold, maxValue, cv::THRESH_BINARY);
    cv::findContours(normImage, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    cv::Mat overlay_image;
    cv_ptr->image.copyTo(overlay_image);
    for(auto c: contours){
        float area = cv::contourArea(c);
        if(area < 100){
            continue;
        }
        std::vector<cv::Point> approx;
        cv::convexHull(c, approx);    
        approxes.push_back(approx);    
        cv::polylines(overlay_image, c, true, cv::Scalar(255, 0, 0), 2, cv::LINE_AA);
    }
    
    for(auto a: approxes){
      cv::Rect rect = cv::boundingRect(a);
      bboxes.push_back(rect);
    }
    cv::Mat overlay_image_1;
    
    int dot_r = 2;
    int count = 0;
    LM_point.resize(bboxes.size());
    BBox_rectangle.resize(bboxes.size());
    cv_ptr->image.copyTo(overlay_image_1);
    for(auto r: bboxes){
        cv::rectangle(overlay_image_1, r.tl(), r.br(), cv::Scalar(0, 0, 0), 2, cv::LINE_AA);
        LM_point[count].resize(2);
        BBox_rectangle[count].resize(4);
        BBox_rectangle[count][0] = int(r.x*2);
        BBox_rectangle[count][1] = int(r.y*1.5);
        BBox_rectangle[count][2] = int((r.x+r.width)*2);
        BBox_rectangle[count][3] = int((r.y+r.height)*1.5);
        LM_point[count][0] = (r.x+r.width/2)*2;
        LM_point[count][1] = (r.y+r.height/2)*1.5;
        count += 1;
        cv::circle(overlay_image_1, LM_position_raw_data, dot_r, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
    }
    flag_bbox = true;
     
}
 
void depth_estimater::depthImageCallback(const sensor_msgs::ImageConstPtr& msg){
    
    try{
        cv_ptr_depth = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        
    }catch (cv_bridge::Exception& ex){
        ROS_ERROR("error");
        exit(-1);
    }

    
    sum_depth.resize(BBox_rectangle.size());
    flag_depth = false;
    int sum_count = 1;
    for(int count = 0;count < BBox_rectangle.size();count++){
        sum_count = 1;
        for(int i = 0; i < cv_ptr_depth->image.rows; i++)
        {
            for(int j = 0; j < cv_ptr_depth->image.cols; j++)
            {   
                uint16_t Di = cv_ptr_depth->image.at<u_int16_t>(i,j);
                if (BBox_rectangle[count][0] + 5 < j && BBox_rectangle[count][1] + 5 < i){
                    if(BBox_rectangle[count][2] - 5 > j && BBox_rectangle[count][3] - 5 > i && Di > 0){
                        sum_depth[count] += Di;
                        sum_count += 1;
                    }

                }

            }

        }
        sum_depth[count] = sum_depth[count]/sum_count;
    }

    flag_depth = true;
    
    cv::Mat img_1 = cv::Mat::zeros(500, 500, CV_8UC3);
    cv::Mat overlay_image;
    cv_ptr_depth->image.copyTo(overlay_image);
    cv::circle(overlay_image, LM_position_raw_data, 2, cv::Scalar(255, 0, 0), -1, cv::LINE_AA);
    cv::circle(img_1, LM_position_raw_data, 2, cv::Scalar(255, 0, 0), -1, cv::LINE_AA);
    
}

void depth_estimater::main(){
    ros::Rate loop_rate(10);
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh,"/saliency/image", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh,"/camera/depth/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::CameraInfo> camera_info_sub(nh,"/camera/depth/camera_info",1);
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image,sensor_msgs::CameraInfo> sync(rgb_sub, depth_sub, camera_info_sub, 1);
    sync.registerCallback(boost::bind(&depth_estimater::camera_info_callback, this, _1, _2, _3));
    while(false == ros::isShuttingDown()){
        
        if(flag_depth && flag_bbox && camera_info_flag && sum_depth.size() > 0){
            tf2_ros::TransformListener tfListener(tfBuffer);
            LM_position_array.position = {};
            for (int i = 0;i < LM_point.size();i++){
                std::string LM_position_st = "lm_position_" ;
                std::ostringstream st;
                st << i ;
                LM_position_st = LM_position_st + st.str();
                LM_position_raw_data.x = LM_point[i][0];
                LM_position_raw_data.y = LM_point[i][1];
                cv::Point2d LM_position_correction_data = cam_model.rectifyPoint(LM_position_raw_data);
                cv::Point3d LM_position_3Ddata = cam_model.projectPixelTo3dRay(LM_position_correction_data);
                
                LM_position_3Ddata.x = LM_position_3Ddata.x*0.001*sum_depth[i];
                LM_position_3Ddata.y = LM_position_3Ddata.y*0.001*sum_depth[i];
                LM_position_3Ddata.z = LM_position_3Ddata.z*0.001*sum_depth[i];

                geometry_msgs::TransformStamped transformStamped;
                transformStamped.header.stamp = ros::Time::now();
                transformStamped.header.frame_id = "camera_depth_optical_frame";
                transformStamped.child_frame_id = LM_position_st;
                transformStamped.transform.translation.x = LM_position_3Ddata.x;
                transformStamped.transform.translation.y = LM_position_3Ddata.y;
                transformStamped.transform.translation.z = LM_position_3Ddata.z;
                tf2::Quaternion q;
                q.setRPY(0, 0, 0.0);
                transformStamped.transform.rotation.x = q.x();
                transformStamped.transform.rotation.y = q.y();
                transformStamped.transform.rotation.z = q.z();
                transformStamped.transform.rotation.w = q.w();
                dynamic_br_.sendTransform(transformStamped);

                geometry_msgs::TransformStamped lookuptransformStamped;
                try{
                        lookuptransformStamped = tfBuffer.lookupTransform(LM_position_st, "map",ros::Time());
                }
                catch (tf2::TransformException &ex){
                        ROS_WARN("%s",ex.what());
                        ros::Duration(1.0).sleep();
                        continue;
                }
                
                LM_position.header.frame_id = LM_position_st;
                LM_position.header.stamp = ros::Time::now();
                LM_position.header.seq = i;
                LM_position.x = LM_position_3Ddata.x;
                LM_position.y = LM_position_3Ddata.y;
                LM_position.z = LM_position_3Ddata.z;
                LM_rviz_publish(LM_position_3Ddata,i);
                LM_position_array.position.push_back(LM_position);
            }
            
            LM_pub.publish(LM_position_array);
                           
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }

}

void depth_estimater::LM_rviz_publish(const cv::Point3d& LM_position_rviz,int count){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "camera_depth_optical_frame";
    marker.header.stamp = ros::Time::now();
    marker.ns = "LM_position";
    marker.id = count;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration();

    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.pose.position.x = LM_position_rviz.x;
    marker.pose.position.y = LM_position_rviz.y;
    marker.pose.position.z = LM_position_rviz.z;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;
    marker_pub.publish(marker);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "depth_estimater");
    depth_estimater depthestimate;
    depthestimate.main();
    
    return 0;
}