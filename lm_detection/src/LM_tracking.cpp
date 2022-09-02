#include <ros/ros.h>
#include <lm_detection/Position_array.h>
#include <lm_detection/Position.h>
#include <sensor_msgs/Image.h>
#include <vector>
#include <iostream>
#include <string>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

class LM_tracking{
    public:
        LM_tracking();
        ~LM_tracking();
        void process();
        void LM_position_callback(const lm_detection::Position_array::ConstPtr&);
        void Camera_callback(const sensor_msgs::CameraInfoConstPtr&);
        void detectionImageCallback(const sensor_msgs::ImageConstPtr&);
        void rgbImageCallback(const sensor_msgs::ImageConstPtr&);
        void trackingcallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr&);
        void depthImageCallback(const sensor_msgs::ImageConstPtr&);
        void camera2worldframe(std::vector<int>&);
        void tf_broadcast_and_lookuptransform(cv::Point3d&,std::string&);
        std::vector<int> depth_estimate();
        lm_detection::Position_array LM_position_array;
        lm_detection::Position LM_position;
        darknet_ros_msgs::BoundingBox box_rectangle;
        darknet_ros_msgs::BoundingBoxes boxes_rectangle;
        darknet_ros_msgs::BoundingBoxes tracking_bounding_boxes;
        std::vector<std::vector <int>> BBox_rectangle;
        cv_bridge::CvImagePtr cv_ptr_rgb;
        cv_bridge::CvImagePtr cv_ptr_depth;
        std::vector<int> sum_depth;
        tf2_ros::TransformBroadcaster dynamic_br_;
        tf2_ros::Buffer tfBuffer;
        bool flag_bbox = false;
        bool flag_track = false;
        bool flag_rgb_image = false;
        image_geometry::PinholeCameraModel cam_model;

    private:
        ros::NodeHandle nh;
        image_transport::ImageTransport it;
        image_transport::Publisher pub_rectangle_image;
        ros::Subscriber sub_tracking_bbox,sub_depth;
        ros::Subscriber sub_camera_info;
        ros::Subscriber sub_detection_map;
        ros::Subscriber sub_rgb;
        ros::Publisher pub_bbox,pub_LM;    
};
    
LM_tracking::LM_tracking()
    : it(nh)
{
    sub_camera_info = nh.subscribe<sensor_msgs::CameraInfo>("camera/depth/camera_info",1,&LM_tracking::Camera_callback,this);
    sub_detection_map = nh.subscribe<sensor_msgs::Image>("saliency/image", 1, &LM_tracking::detectionImageCallback,this);
    sub_rgb = nh.subscribe<sensor_msgs::Image>("camera/color/image_raw", 1, &LM_tracking::rgbImageCallback,this);
    sub_tracking_bbox = nh.subscribe<darknet_ros_msgs::BoundingBoxes>("tracking_data/bounding_boxes", 1,&LM_tracking::trackingcallback,this);
    sub_depth = nh.subscribe<sensor_msgs::Image>("/camera/depth/image_raw", 5, &LM_tracking::depthImageCallback, this);
    pub_rectangle_image = it.advertise("image_rectangle",1);
    pub_LM = nh.advertise<lm_detection::Position_array>("lm_position_current",1);
    pub_bbox = nh.advertise<darknet_ros_msgs::BoundingBoxes>("bounding_boxes",1);
}

LM_tracking::~LM_tracking(){

}

void LM_tracking::Camera_callback(const sensor_msgs::CameraInfoConstPtr& cam_info_msg){
    cam_model.fromCameraInfo(cam_info_msg);
}


void LM_tracking::trackingcallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg){
    tracking_bounding_boxes = *msg;
    if(!tracking_bounding_boxes.bounding_boxes.empty()){
        flag_track = true;
    }
}

void LM_tracking::depthImageCallback(const sensor_msgs::ImageConstPtr& msg){

    try{
        cv_ptr_depth = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        
    }catch (cv_bridge::Exception& ex){
        ROS_ERROR("error");
        exit(-1);
    }

}

void LM_tracking::rgbImageCallback(const sensor_msgs::ImageConstPtr& msg){
    
    try{
        cv_ptr_rgb = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    }catch (cv_bridge::Exception& ex){
        ROS_ERROR("error");
        exit(-1);
    }
    
    flag_rgb_image = true;
}

void LM_tracking::detectionImageCallback(const sensor_msgs::ImageConstPtr& msg){
 

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
    BBox_rectangle.resize(bboxes.size());
    cv_ptr->image.copyTo(overlay_image_1);
    for(auto r: bboxes){
        cv::rectangle(overlay_image_1, r.tl(), r.br(), cv::Scalar(0, 0, 0), 2, cv::LINE_AA);
        BBox_rectangle[count].resize(4);
        BBox_rectangle[count][0] = r.x;
        BBox_rectangle[count][1] = r.y;
        BBox_rectangle[count][2] = r.x+r.width;
        BBox_rectangle[count][3] = r.y+r.height;
        count += 1;
    }
    flag_bbox = true;
    std::cout << "BBox subscribe !!" << std::endl;
     
}

std::vector<int> LM_tracking::depth_estimate(){
    
    sum_depth.resize(tracking_bounding_boxes.bounding_boxes.size());
    int sum_count = 1;
    for(int count = 0;count < tracking_bounding_boxes.bounding_boxes.size();count++){
        sum_count = 1;
        for(int i = 0; i < cv_ptr_depth->image.rows; i++)
        {
            for(int j = 0; j < cv_ptr_depth->image.cols; j++)
            {   
                uint16_t Di = cv_ptr_depth->image.at<u_int16_t>(i,j);
                if (tracking_bounding_boxes.bounding_boxes[count].xmin*2 + 5 < j && int(tracking_bounding_boxes.bounding_boxes[count].ymin*1.5 + 5) < i){
                    if(tracking_bounding_boxes.bounding_boxes[count].xmax*2 - 5 > j && int(tracking_bounding_boxes.bounding_boxes[count].ymax*1.5 - 5) > i && Di > 0){
                        sum_depth[count] += Di;
                        sum_count += 1;
                    }

                }

            }

        }
        sum_depth[count] = sum_depth[count]/sum_count;
    }

    return sum_depth;
}

void LM_tracking::camera2worldframe(std::vector<int>& sum_depth){
    cv::Point2d LM_position_raw_data;
    LM_position_array.position = {};
    for(int i = 0;i < sum_depth.size();i++){
        std::string LM_position_st = "lm_current_position_" ;
        std::ostringstream st;
        st << i ;
        LM_position_st = LM_position_st + st.str();
        LM_position_raw_data.x = int((tracking_bounding_boxes.bounding_boxes[i].xmin*0.5 + tracking_bounding_boxes.bounding_boxes[i].xmax*0.5)*2);
        LM_position_raw_data.y = int((tracking_bounding_boxes.bounding_boxes[i].ymin*0.5 + tracking_bounding_boxes.bounding_boxes[i].ymax*0.5)*1.5);
        cv::Point2d LM_position_correction_data = cam_model.rectifyPoint(LM_position_raw_data);
        cv::Point3d LM_position_3Ddata = cam_model.projectPixelTo3dRay(LM_position_correction_data);
        LM_position_3Ddata.x = LM_position_3Ddata.x*0.001*sum_depth[i];
        LM_position_3Ddata.y = LM_position_3Ddata.y*0.001*sum_depth[i];
        LM_position_3Ddata.z = LM_position_3Ddata.z*0.001*sum_depth[i];
        tf_broadcast_and_lookuptransform(LM_position_3Ddata,LM_position_st);
    }
    pub_LM.publish(LM_position_array);
}

void LM_tracking::tf_broadcast_and_lookuptransform(cv::Point3d& LM_position_3Ddata,std::string& LM_position_st){
    
    tf2_ros::TransformListener tfListener(tfBuffer);
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
            lookuptransformStamped = tfBuffer.lookupTransform("map",LM_position_st,ros::Time(0));
    }
    catch (tf2::TransformException &ex){
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            return;
    }

    auto& trans = lookuptransformStamped.transform.translation;
    LM_position.header.frame_id = LM_position_st;
    LM_position.header.stamp = ros::Time(0);
    LM_position.x = trans.x;
    LM_position.y = trans.y;
    LM_position.z = trans.z;
    LM_position_array.position.push_back(LM_position);
}

void LM_tracking::process(){
    ros::Rate loop_rate(50);
    while(false == ros::isShuttingDown()){
        if(flag_bbox && flag_rgb_image){
            boxes_rectangle.bounding_boxes = {};
            boxes_rectangle.header.frame_id = "camera_depth_optical_frame";
            boxes_rectangle.header.stamp = ros::Time::now();

            for(int i = 0;i<BBox_rectangle.size();i++){
                box_rectangle.xmin = BBox_rectangle[i][0];
                box_rectangle.ymin = BBox_rectangle[i][1];
                box_rectangle.xmax = BBox_rectangle[i][2];
                box_rectangle.ymax = BBox_rectangle[i][3];
                box_rectangle.probability = 0.9;
                boxes_rectangle.bounding_boxes.push_back(box_rectangle);
                cv::rectangle(cv_ptr_rgb->image,cv::Point(BBox_rectangle[i][0],BBox_rectangle[i][1]),cv::Point(BBox_rectangle[i][2],BBox_rectangle[i][3]), cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
            }

            
            pub_rectangle_image.publish(cv_ptr_rgb->toImageMsg());
            pub_bbox.publish(boxes_rectangle);

            if(flag_track){
                std::cout << "Size of tracking -> " << tracking_bounding_boxes.bounding_boxes.size() << std::endl;
                sum_depth = depth_estimate();
                camera2worldframe(sum_depth);
            }

        }
        ros::spinOnce();
        loop_rate.sleep();
    }

}
    


int main(int argc, char **argv){
    ros::init(argc, argv, "LM_tracking");
    LM_tracking LM_tracking;
    LM_tracking.process();
    return 0;
}