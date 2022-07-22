#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <iostream>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Header.h>
#include<vector>
#include<string>
#include<lm_detection/Position.h>
#include<lm_detection/Position_array.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include<math.h>
#include <sstream>
#include <typeinfo>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/LU>

class depth_estimater{
public:
    depth_estimater();
    ~depth_estimater();
    void rgbImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void depthImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void main();
    cv::Point dot_P;
    tf2_ros::TransformBroadcaster dynamic_br_;
    tf2_ros::Buffer tfBuffer;
    std::vector<std::vector <double>> LM_point;
    std::vector<std::vector <int>> BBox_rectangle;
    lm_detection::Position_array LM_position_array;
    lm_detection::Position LM_position;
    cv_bridge::CvImagePtr cv_ptr_depth;
    std::vector<int> sum_depth;
    bool flag_depth = false;
    bool flag_bbox = false;
    std_msgs::Float32MultiArray array;
    ros::Publisher LM_pub;
    
    // tf::TransformListener listener;
private:
    ros::NodeHandle nh;
    ros::Subscriber sub_rgb, sub_depth;
    
};
 
depth_estimater::depth_estimater(){
    sub_rgb = nh.subscribe<sensor_msgs::Image>("/saliency/image", 5, &depth_estimater::rgbImageCallback, this);
    sub_depth = nh.subscribe<sensor_msgs::Image>("/camera/depth/image_raw", 5, &depth_estimater::depthImageCallback, this);
    LM_pub = nh.advertise<lm_detection::Position_array>("lm_position",10);
}
 
depth_estimater::~depth_estimater(){
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
    const double threshold = 100.0;
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
        dot_P.x = r.x+r.width/2;
        dot_P.y = r.y+r.height/2;
        LM_point[count][0] = (r.x+r.width/2)*2;
        LM_point[count][1] = (r.y+r.height/2)*1.5;
        count += 1;
        cv::circle(overlay_image_1, dot_P, dot_r, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
    }
    flag_bbox = true;
    
    cv::imshow("BBox image", overlay_image_1);  
    cv::waitKey(1);  
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
    // double max_range_ = 20;
    // double min_range_ = 0.5;
    for(int count = 0;count < BBox_rectangle.size();count++){
        for(int i = 0; i < cv_ptr_depth->image.rows; i++)
        {
            for(int j = 0; j < cv_ptr_depth->image.cols; j++)
            {   
                int Di = cv_ptr_depth->image.at<u_int16_t>(i,j);
                if (BBox_rectangle[count][0] < j && BBox_rectangle[count][1] < i){
                    if(BBox_rectangle[count][2] > j && BBox_rectangle[count][3] > i && Di > 0){
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
    cv::circle(overlay_image, dot_P, 2, cv::Scalar(255, 0, 0), -1, cv::LINE_AA);
    cv::circle(img_1, dot_P, 2, cv::Scalar(255, 0, 0), -1, cv::LINE_AA);
    
    cv::imshow("Depth image", overlay_image);
    cv::waitKey(1);
}

void depth_estimater::main(){
    ros::Rate loop_rate(10);
    
    while(false == ros::isShuttingDown()){
        
        if(flag_depth && flag_bbox && sum_depth.size() > 0){
            tf2_ros::TransformListener tfListener(tfBuffer);
            double cx = 720/2;
            double cy = 1280/2;
            double fov = 85.2*0.01745329251994329577;
            double fx = 1.0/(2.0*tan(fov/2)) * 720;
            Eigen::Matrix<double,3,3> K;
            std::cout << "LM point = " << LM_point.size() << std::endl;
            std::cout << "BBox rec = " << sum_depth.size() << std::endl;
            K << fx,0,cx,
                 0,fx,cy,
                 0,0,1;
            Eigen::Matrix<double,3,3> K_inv = K.inverse();
            for (int i = 0;i < LM_point.size();i++){
                std::string LM_position_st = "lm_position_" ;
                std::ostringstream st;
                st << i;
                LM_position_st = LM_position_st + st.str();
                
                Eigen::Vector3d LM;
                LM << LM_point[i][0]-cx,LM_point[i][1]-cy,1.0;
                Eigen::Vector3d LM_;
                LM_ = LM * sum_depth[i];
                Eigen::Vector3d LM_trans;   
                LM_trans = K_inv * LM_;

                LM_trans(0) = LM_trans(0)*0.01/fx;
                LM_trans(1) = LM_trans(1)*0.01/fx;               
                LM_trans(2) = LM_trans(2)*0.001;

                std::cout << "X = " << LM_trans(0) << std::endl;
                std::cout << "Y = " << LM_trans(1) << std::endl;
                std::cout << "Z = " << LM_trans(2) << std::endl;

                geometry_msgs::TransformStamped transformStamped;
                transformStamped.header.stamp = ros::Time::now();
                transformStamped.header.frame_id = "camera_depth_optical_frame";
                transformStamped.child_frame_id = LM_position_st;
                transformStamped.transform.translation.x = LM_trans(0);
                transformStamped.transform.translation.y = LM_trans(1);
                transformStamped.transform.translation.z = LM_trans(2);
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
                auto& map_to_depth_transform = lookuptransformStamped.transform.translation;
                LM_position.header.frame_id = LM_position_st;
                LM_position.header.stamp = ros::Time::now();
                LM_position.x = map_to_depth_transform.x;
                LM_position.y = map_to_depth_transform.y;
                LM_position.z = map_to_depth_transform.z;
                LM_position_array.position.push_back(LM_position);
            }

            LM_pub.publish(LM_position_array);
                           
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }

}
 
int main(int argc, char **argv){
    ros::init(argc, argv, "depth_estimater");
 
    depth_estimater depthestimate;
    depthestimate.main();
    
    return 0;
}