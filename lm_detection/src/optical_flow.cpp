#include "LM_detection/optical_flow.h"

optical_flow::optical_flow()
    : it(nh)
{
    rgb_sub = nh.subscribe<sensor_msgs::Image>("camera/color/image_raw", 1, &optical_flow::rgbImageCallback,this);
    bbox_sub = nh.subscribe("detection/bbox_array",1,&optical_flow::BboxCallback,this);
    bbox_continue_sub = nh.subscribe("detection/bbox_continue_array",1,&optical_flow::Bbox_continue_Callback,this);
    bbox_pub = nh.advertise<lm_detection::Bounding_Box_array>("tracking/flow_bbox",1);
    flow_pub_image = it.advertise("tracking/flow_image",1);
    bbox_image_pub = it.advertise("tracking/bbox_image",1);
}

void optical_flow::rgbImageCallback(const sensor_msgs::ImageConstPtr& msg){
    // 8bitで入力制限、オプティカルフローにより変更可能
    try{
        rgb_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
        subscribe_rgbimage = true;
    }catch (cv_bridge::Exception& ex){
        ROS_ERROR("error");
        exit(-1);
    }
    ROS_INFO("get image");
    
}

void optical_flow::BboxCallback(const lm_detection::Bounding_Box_array& msg){
    bbox_initial = msg;
    if(bbox_initial.bbox.size() > 0){
        subscribe_bbox = true;
        ROS_INFO("get bbox");
    }   
}

void optical_flow::Bbox_continue_Callback(const lm_detection::Bounding_Box_array& msg){
    bbox_continue = msg;  
    std::cout << bbox_continue <<std::endl;
    if(bbox_continue.bbox.size() > 0){
        subscribe_bbox = true;
        ROS_INFO("get bbox");
    }   
}

void optical_flow::process(){
    flag_first_frame = true;
    ros::Rate loop_rate(200);
    
    while(false == ros::isShuttingDown()){
        if(subscribe_rgbimage&&subscribe_bbox){

            if(flag_first_frame){
                rgb_image->image.copyTo(prev_image);
                flag_first_frame = false;
                prev_bbox = bbox_initial;
                all_flow = cv::Mat::zeros(prev_image.size(), CV_32FC2);
            }else{
                rgb_image->image.copyTo(next_image);
                // 入力画像の準備
                cv::Mat flow(prev_image.size(), CV_32FC2);
                cv::cvtColor(prev_image,prev_gray_image,cv::COLOR_BGR2GRAY);
                cv::cvtColor(next_image,next_gray_image,cv::COLOR_BGR2GRAY);

                // オプティカルフローの計算
                cv::calcOpticalFlowFarneback(prev_gray_image, next_gray_image, flow, 0.8, 10, 15, 3, 5, 1.1, 0);
                
                // オプティカルフローの表示
                std::cout << next_image.size() << std::endl;
                cv::split(flow, flow_parts);
                cv::cartToPolar(flow_parts[0], flow_parts[1], magnitude, angle, true);
                cv::normalize(magnitude, magn_norm, 0.0f, 1.0f, cv::NORM_MINMAX);               
                angle *= ((1.f / 360.f) * (180.f / 255.f));   
                _hsv[0] = angle;
                _hsv[1] = cv::Mat::ones(angle.size(), CV_32F);
                _hsv[2] = magn_norm;
                cv::merge(_hsv, 3, hsv);
                hsv.convertTo(hsv8, CV_8U, 255.0);
                cv::cvtColor(hsv8, bgr, cv::COLOR_HSV2BGR);
                sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", bgr).toImageMsg();
                flow_pub_image.publish(msg);
                cv::imshow("frame2", bgr);

                // 次のバウンディングボックスを算出,publish
                next_bbox = Next_Bounding_Box_output(prev_bbox,flow_parts);
                
                bbox_pub.publish(next_bbox);
                
                std::swap(prev_image,next_image);
                prev_bbox = next_bbox;
                flag_first_frame = false;
            }
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }
              
}

lm_detection::Bounding_Box_array optical_flow::Next_Bounding_Box_output(const lm_detection::Bounding_Box_array& prev_bbox,cv::Mat flow_[2]){
    lm_detection::Bounding_Box_array calc_next_bbox = prev_bbox;
    flow_[0].convertTo(flow_x, CV_8U, 255.0);
    flow_[1].convertTo(flow_y, CV_8U, 255.0);
    for(int count = 0;count < prev_bbox.bbox.size();count++){
        float bbox_flow_x = 0;
        float bbox_flow_y = 0;
        int num = 0;
        for(int i = 0;i < flow_x.rows;i++){
            for(int j = 0;j < flow_x.cols;j++){
                if(prev_bbox.bbox[count].xmin  < j - 5 && prev_bbox.bbox[count].ymin  < i - 5 && prev_bbox.bbox[count].xmax  > j + 5 && prev_bbox.bbox[count].ymax  > i + 5 && flow_[0].at<float>(i,j) != 0 && flow_[1].at<float>(i,j) != 0){
                    bbox_flow_x += flow_[0].at<float>(i,j);
                    bbox_flow_y += flow_[1].at<float>(i,j);    
                    num++;
                }
            }
        }
        if(num == 0){
            calc_next_bbox.bbox[count] = prev_bbox.bbox[count];
        }else{
            calc_next_bbox.bbox[count].xmin = int(prev_bbox.bbox[count].xmin + bbox_flow_x/num);
            calc_next_bbox.bbox[count].ymin = int(prev_bbox.bbox[count].ymin + bbox_flow_y/num);
            calc_next_bbox.bbox[count].xmax = int(prev_bbox.bbox[count].xmax + bbox_flow_x/num);
            calc_next_bbox.bbox[count].ymax = int(prev_bbox.bbox[count].ymax + bbox_flow_y/num);
            calc_next_bbox.bbox[count].id = prev_bbox.bbox[count].id;
        }
        iou = IoU(calc_next_bbox.bbox[count],bbox_continue.bbox[count]);
        std::cout << "IoU value : " << iou << std::endl;
        if(iou > 0.6){
            calc_next_bbox.bbox[count] = bbox_continue.bbox[count];
        }

        cv::rectangle(rgb_image->image,cv::Point(bbox_continue.bbox[count].xmin,bbox_continue.bbox[count].ymin),cv::Point(bbox_continue.bbox[count].xmax,bbox_continue.bbox[count].ymax ), cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
        cv::rectangle(rgb_image->image,cv::Point(calc_next_bbox.bbox[count].xmin,calc_next_bbox.bbox[count].ymin),cv::Point(calc_next_bbox.bbox[count].xmax,calc_next_bbox.bbox[count].ymax ), cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
        
    }
    
    bbox_image_pub.publish(rgb_image->toImageMsg());
    return calc_next_bbox;
}

float optical_flow::IoU(const lm_detection::Bounding_Box& calc_bbox,const lm_detection::Bounding_Box& detecte_bbox){
    float dx = std::min(calc_bbox.xmax,detecte_bbox.xmax) - std::max(calc_bbox.xmin,detecte_bbox.xmin);
    float dy = std::min(calc_bbox.ymax,detecte_bbox.ymax) - std::max(calc_bbox.ymin,detecte_bbox.ymin);
    float calc_mass = (calc_bbox.xmax-calc_bbox.xmin)*(calc_bbox.ymax - calc_bbox.ymin);
    float detecte_mass = (detecte_bbox.xmax-detecte_bbox.xmin)*(detecte_bbox.ymax - detecte_bbox.ymin);
    float iou = dx*dy/(calc_mass+detecte_mass-dx*dy);
    return iou;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "optical_flow");
    optical_flow optical_flow;
    optical_flow.process();
    return 0;
}