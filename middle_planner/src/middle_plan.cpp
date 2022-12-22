#include "middle_planner/middle_plan.h"

CDCPlanner::CDCPlanner(void)
:wp_flag(false),lm_flag(false)
{
    wp_init_sub = nh.subscribe("/waypoint",1,&CDCPlanner::wp_callback,this);
    lm_current_sub = nh.subscribe("tracking/lm_position_current",1,&CDCPlanner::lm_current_callback,this);
    lm_first_sub = nh.subscribe("detection/lm_first_position",1,&CDCPlanner::lm_first_callback,this);
    wp_pub = nh.advertise<waypoint_generator::Waypoint_array>("waypoint_new", 1);
    marker_pub = nh.advertise<visualization_msgs::Marker>("waypoint_marker_new", 1);
}


void CDCPlanner::wp_callback(const waypoint_generator::Waypoint_array& msg)
{
    wp_array_subscribe = msg;
    if(wp_array_subscribe.wp.size() > 0){
        wp_flag = true;
    }
    
}

void CDCPlanner::lm_current_callback(const lm_detection::Position_array& msg)
{
    lm_current_array = msg;
    if(lm_current_array.position.size() > 2){
        lm_flag = true;
    }
    
}

void CDCPlanner::lm_first_callback(const lm_detection::Position_array& msg)
{
    lm_first_array = msg;
}

void CDCPlanner::process(void)
{
    ros::Rate loop_rate(10);
    int first_flag = 0;
    ROS_INFO("       middle plan start        ");
    ROS_INFO("================================");
    while(ros::ok()){
        if(wp_flag && lm_flag){
            
            prepare();
            LM_current = Eigen::MatrixXd::Ones(3,lm_current_array.position.size());
            std::vector<std::string> LM_current_number(lm_current_array.position.size());

            for (int i = 0; i < lm_current_array.position.size(); i++){
                LM_current(0,i) = lm_current_array.position[i].x;
                LM_current(1,i) = lm_current_array.position[i].y;
                LM_current_number[i] = lm_current_array.position[i].header.seq;
                // ROS_INFO("lm_current get!! %f %f ",LM_current(0,i),LM_current(1,i));
            }
            
            LM_first_matching = LM_first_matching_current(LM_first);
            wp_new = A_matrix(LM_first_matching,LM_current,wp_first);
            WP_publish(wp_new);
            
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

Eigen::MatrixXd CDCPlanner::LM_first_matching_current(const Eigen::MatrixXd& LM_first)
{   
    LM_first_stock = Eigen::MatrixXd::Ones(3,LM_current.cols());
    for (int i = 0;i < LM_current.cols();i++){       
        auto itr = std::find(LM_first_number.begin(),LM_first_number.end(),LM_current_number[i]);        
        const int wanted_index = std::distance(LM_first_number.begin(), itr);
        LM_first_stock(0,i) =  LM_first(0,wanted_index);
        LM_first_stock(1,i) =  LM_first(1,wanted_index);        
    }
    return LM_first_stock;
}

void CDCPlanner::prepare(void)
{
    wp_first = Eigen::MatrixXd::Ones(3,wp_array_subscribe.wp.size());
    for (int i = 0;i<wp_array_subscribe.wp.size();i++){
        wp_first(0,i) = wp_array_subscribe.wp[i].x;
        wp_first(1,i) = wp_array_subscribe.wp[i].y;
        // ROS_INFO("wp_init get!! %f %f ",wp_first(0,i),wp_first(1,i));
    }

    LM_first = Eigen::MatrixXd::Ones(3,lm_first_array.position.size());
    std::vector<std::string> LM_first_number(lm_first_array.position.size());
    for (int i = 0;i<lm_first_array.position.size();i++){
        LM_first(0,i) = lm_first_array.position[i].x;
        LM_first(1,i) = lm_first_array.position[i].y;
        LM_first_number[i] = lm_first_array.position[i].header.seq;
        // ROS_INFO("lm_first get!! %f %f ",LM_first(0,i),LM_first(1,i));
    }
    return;
}

Eigen::MatrixXd CDCPlanner::A_matrix(const Eigen::MatrixXd& lm_first,const Eigen::MatrixXd& lm_current,const Eigen::MatrixXd& wp_first)
{
    Eigen::MatrixXd A;
    double epsilon = 0.000001;
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(lm_first,Eigen::ComputeThinU | Eigen::ComputeThinV);
    double tolerance = epsilon * std::max(lm_first.cols(), lm_first.rows()) *svd.singularValues().array().abs()(0);
    Eigen::MatrixXd lm_first_inverse =  svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
    A = lm_current * lm_first_inverse;
    wp_new =  A * wp_first;
    return wp_new;
}

void CDCPlanner::WP_publish(Eigen::MatrixXd& wp_new)
{
    for (int i = 0;i < wp_new.cols();i++){
        wp.header.frame_id = "map";
        wp.header.stamp = ros::Time::now();
        wp.header.seq = i;
        wp.x = wp_new(0,i);
        wp.y = wp_new(1,i);
        wp_array_publish.wp.push_back(wp);

        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "waypoint_new";
        marker.id = i;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration();

        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.2;
        marker.pose.position.x = wp_new(0,i);
        marker.pose.position.y = wp_new(1,i);
        marker.pose.position.z = 0;
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
    // ROS_ERROR("Publish New Waypoint !!");  
    wp_pub.publish(wp_array_publish);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cdc_planner");
    CDCPlanner planner;
    planner.process();
    return 0;
}