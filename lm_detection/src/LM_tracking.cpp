#include "LM_detection/LM_tracking.h"
    
LM_tracking::LM_tracking()
{
    sub_camera_info = nh.subscribe<sensor_msgs::CameraInfo>("camera/depth/camera_info",1,&LM_tracking::Camera_callback,this);
    sub_tracking_bbox = nh.subscribe("tracking/flow_bbox", 1,&LM_tracking::trackingcallback,this);
    sub_depth = nh.subscribe<sensor_msgs::Image>("/camera/depth/image_raw", 5, &LM_tracking::depthImageCallback, this);
    pub_LM_current = nh.advertise<lm_detection::Position_array>("tracking/lm_position_current",1);
    marker_pub = nh.advertise<visualization_msgs::Marker>("tracking/lm_current_position_marker", 1);
}

LM_tracking::~LM_tracking(){

}

void LM_tracking::Camera_callback(const sensor_msgs::CameraInfoConstPtr& cam_info_msg){
    cam_model.fromCameraInfo(cam_info_msg);
}


void LM_tracking::trackingcallback(const lm_detection::Bounding_Box_array& msg){
    tracking_bounding_boxes = msg;
    if(tracking_bounding_boxes.bbox.size() > 2){
        flag_track = true;
        std::cout << "get tracking bbox !!" << tracking_bounding_boxes.bbox.size() << std::endl;
        bbox_number = tracking_bounding_boxes.bbox.size();
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


std::vector<int> LM_tracking::depth_estimate(){
    
    sum_depth.resize(tracking_bounding_boxes.bbox.size());
    int sum_count = 1;
    for(int count = 0;count < tracking_bounding_boxes.bbox.size();count++){
        sum_count = 1;
        for(int i = 0; i < cv_ptr_depth->image.rows; i++)
        {
            for(int j = 0; j < cv_ptr_depth->image.cols; j++)
            {   
                uint16_t Di = cv_ptr_depth->image.at<u_int16_t>(i,j);
                if (tracking_bounding_boxes.bbox[count].xmin*2 + 5 < j && int(tracking_bounding_boxes.bbox[count].ymin*1.5 + 5) < i){
                    if(tracking_bounding_boxes.bbox[count].xmax*2 - 5 > j && int(tracking_bounding_boxes.bbox[count].ymax*1.5 - 5) > i && Di > 0){
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

void LM_tracking::camera2worldframe(std::vector<int>& sum_depth,std::string& LM_position_st){
    cv::Point2d LM_position_raw_data;
    std::string LM_miss_current_position = "LM_miss_current_position_";
    LM_position_array.position = {};
    for(int i = 0;i < bbox_number;i++){

        LM_position_raw_data.x = int((tracking_bounding_boxes.bbox[i].xmin*0.5 + tracking_bounding_boxes.bbox[i].xmax*0.5)*2);
        LM_position_raw_data.y = int((tracking_bounding_boxes.bbox[i].ymin*0.5 + tracking_bounding_boxes.bbox[i].ymax*0.5)*1.5);
        cv::Point2d LM_position_correction_data = cam_model.rectifyPoint(LM_position_raw_data);
        cv::Point3d LM_position_3Ddata = cam_model.projectPixelTo3dRay(LM_position_correction_data);

        if(tracking_bounding_boxes.bbox[i].xmin > 640 || tracking_bounding_boxes.bbox[i].xmax < 0 || tracking_bounding_boxes.bbox[i].ymin > 480 || tracking_bounding_boxes.bbox[i].ymax < 0){

            std::ostringstream st;
            st << tracking_bounding_boxes.bbox[i].id;
            LM_position_st = LM_miss_current_position + st.str();
            prev_LM_position_array.position[i].header.frame_id = LM_position_st;
            LM_position_array.position.push_back(prev_LM_position_array.position[i]);
        }else{

            std::ostringstream st;
            st << tracking_bounding_boxes.bbox[i].id;
            LM_position_st = LM_position_st + st.str();
            LM_position_3Ddata.x = LM_position_3Ddata.x*0.001*sum_depth[i];
            LM_position_3Ddata.y = LM_position_3Ddata.y*0.001*sum_depth[i];
            LM_position_3Ddata.z = LM_position_3Ddata.z*0.001*sum_depth[i];
            tf_broadcast_and_lookuptransform(LM_position_3Ddata,LM_position_st,tracking_bounding_boxes.bbox[i].id);

        }
            
    }
    std::cout << "prev bbox array" << std::endl;
    prev_LM_position_array = LM_position_array;
    std::cout << prev_LM_position_array << std::endl;
    pub_LM_current.publish(LM_position_array);
    
}

void LM_tracking::tf_broadcast_and_lookuptransform(cv::Point3d& LM_position_3Ddata,std::string& LM_position_st,int count_id){
    
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
    LM_position.header.seq = count_id;
    LM_position.header.stamp = ros::Time(0);
    LM_position.x = trans.x;
    LM_position.y = trans.y;
    LM_position.z = trans.z;
    LM_position_array.position.push_back(LM_position);
}

void LM_tracking::LM_rviz_publish(const lm_detection::Position_array& LM_position_array){
    for(int count = 1;count <= LM_position_array.position.size();count++){
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time(0);
        marker.ns = "LM_current_position";
        marker.id = count;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration();

        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.pose.position.x = LM_position_array.position[count-1].x;
        marker.pose.position.y = LM_position_array.position[count-1].y;
        marker.pose.position.z = LM_position_array.position[count-1].z;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 1;
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;
        marker_pub.publish(marker);
    }
}


void LM_tracking::process(){
    ros::Rate loop_rate(100);
    bool flag = true;
    prev_LM_position_array.position = {};
    while(false == ros::isShuttingDown()){
        if(flag_track){
            if(flag){
                prev_LM_position_array.position.resize(bbox_number);
            }
            sum_depth = depth_estimate();
            std::string lm_string_name = "lm_current_position_";          
            camera2worldframe(sum_depth,lm_string_name);     
            LM_rviz_publish(LM_position_array);
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