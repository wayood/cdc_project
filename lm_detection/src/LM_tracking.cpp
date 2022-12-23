#include "LM_detection/LM_tracking.h"
    
LM_tracking::LM_tracking()
{
    sub_camera_info = nh.subscribe<sensor_msgs::CameraInfo>("camera/depth/camera_info",1,&LM_tracking::Camera_callback,this);
    sub_tracking_bbox = nh.subscribe("tracking/flow_bbox", 1,&LM_tracking::trackingcallback,this);
    sub_depth = nh.subscribe<sensor_msgs::Image>("/camera/depth/image_raw", 5, &LM_tracking::depthImageCallback, this);
    pub_LM_current = nh.advertise<lm_detection::Position_array>("tracking/lm_position_current",1);
}

LM_tracking::~LM_tracking(){

}

void LM_tracking::Camera_callback(const sensor_msgs::CameraInfoConstPtr& cam_info_msg){
    cam_model.fromCameraInfo(cam_info_msg);
}


void LM_tracking::trackingcallback(const lm_detection::Bounding_Box_array& msg){
    tracking_bounding_boxes = msg;
    if(!tracking_bounding_boxes.bbox.empty()){
        flag_track = true;
        std::cout << "get tracking bbox !!" << std::endl;
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
    LM_position_array.position = {};
    for(int i = 0;i < sum_depth.size();i++){
        std::ostringstream st;
        st << tracking_bounding_boxes.bbox[i].id ;
        LM_position_st = LM_position_st + st.str();
        LM_position_raw_data.x = int((tracking_bounding_boxes.bbox[i].xmin*0.5 + tracking_bounding_boxes.bbox[i].xmax*0.5)*2);
        LM_position_raw_data.y = int((tracking_bounding_boxes.bbox[i].ymin*0.5 + tracking_bounding_boxes.bbox[i].ymax*0.5)*1.5);
        cv::Point2d LM_position_correction_data = cam_model.rectifyPoint(LM_position_raw_data);
        cv::Point3d LM_position_3Ddata = cam_model.projectPixelTo3dRay(LM_position_correction_data);
        LM_position_3Ddata.x = LM_position_3Ddata.x*0.001*sum_depth[i];
        LM_position_3Ddata.y = LM_position_3Ddata.y*0.001*sum_depth[i];
        LM_position_3Ddata.z = LM_position_3Ddata.z*0.001*sum_depth[i];
        tf_broadcast_and_lookuptransform(LM_position_3Ddata,LM_position_st);
    }
    
    pub_LM_current.publish(LM_position_array);
    
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
    ros::Rate loop_rate(100);
    int count = 0;
    bool flag = false;
    while(false == ros::isShuttingDown()){
        if(flag_track){
            sum_depth = depth_estimate();
            ROS_INFO("depth get !!");
            std::string lm_string_name = "lm_current_position_";          
            camera2worldframe(sum_depth,lm_string_name);     
            ROS_INFO("publish msg !!");
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