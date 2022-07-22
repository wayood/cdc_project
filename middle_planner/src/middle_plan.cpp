#include "middle_planner/middle_plan.h"

CDCPlanner::CDCPlanner(void)
:wp_flag(false),lm_flag(false)
{
    wp_init_sub = nh.subscribe("/waypoint",1,&CDCPlanner::wp_callback,this);
    lm_sub = nh.subscribe("/lm_position",1,&CDCPlanner::lm_callback,this);
    wp_pub = nh.advertise<waypoint_generator::Waypoint_array>("waypoint_new", 1);
    marker_pub = nh.advertise<visualization_msgs::Marker>("marker", 1);
}


void CDCPlanner::wp_callback(const waypoint_generator::Waypoint_array& msg)
{
    wp_array_subscribe = msg;
    wp_flag = true;
}

void CDCPlanner::lm_callback(const lm_detection::Position_array& msg)
{
    lm_array = msg;
    lm_flag = true;
}

void CDCPlanner::process(void)
{
    ros::Rate loop_rate(10);
    int first_flag = 0;
    ROS_INFO("       middle plan start        ");
    ROS_INFO("================================");
    while(ros::ok()){
        if(wp_flag && lm_flag){
            if(first_flag == 0)
            {
                prepare();
                if(wp_first.cols() > 0 && LM_first.cols() > 2){
                    first_flag = 1;
                }
            }
            std::cout << "LM initial size => " << LM_first.cols() << std::endl;
            std::cout << "WayPoint initial size => " << wp_first.cols() << std::endl;
            if(first_flag == 1)
            {
                LM_current = Eigen::MatrixXd::Ones(3,lm_array.position.size());
                for (int i = 0; i < lm_array.position.size(); i++){
                    LM_current(0,i) = lm_array.position[i].x;
                    LM_current(1,i) = lm_array.position[i].y;
                    ROS_INFO("lm_current get!! %f %f ",LM_current(0,i),LM_current(1,i));
                }
                

                if(LM_current.cols() == LM_first.cols()){
                    ROS_ERROR("calucurate A Matrix !!");
                    wp_new = A_matrix(LM_first,LM_current,wp_first);
                    WP_publish(wp_new);
                }
                
            }
            
            
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void CDCPlanner::prepare(void)
{
    wp_first = Eigen::MatrixXd::Ones(3,wp_array_subscribe.wp.size());
    for (int i = 0;i<wp_array_subscribe.wp.size();i++){
        wp_first(0,i) = wp_array_subscribe.wp[i].x;
        wp_first(1,i) = wp_array_subscribe.wp[i].y;
        // ROS_INFO("wp_init get!! %f %f ",wp_first(0,i),wp_first(1,i));
    }

    LM_first = Eigen::MatrixXd::Ones(3,lm_array.position.size());
    for (int i = 0;i<lm_array.position.size();i++){
        LM_first(0,i) = lm_array.position[i].x;
        LM_first(1,i) = lm_array.position[i].y;
        // ROS_INFO("lm_first get!! %f %f ",LM_first(0,i),LM_first(1,i));
    }
    return;
}

Eigen::MatrixXd CDCPlanner::A_matrix(const Eigen::MatrixXd& lm_first,const Eigen::MatrixXd& lm_current,const Eigen::MatrixXd& wp_first)
{
    Eigen::MatrixXd A;
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(lm_first,Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd s = svd.singularValues();
    s = s.array().inverse();
    Eigen::MatrixXd lm_first_inverse = svd.matrixV() * s.asDiagonal() * svd.matrixU().transpose();
    A = lm_current * lm_first_inverse;
    wp_new =  A * wp_first;
    ROS_ERROR("return New Waypoint !!");
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
        marker.ns = "basic_shapes";
        marker.id = i;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration();

        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.2;
        marker.pose.position.x = wp_new(0,i);
        marker.pose.position.y = wp_new(1,i);
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 1;
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;
        marker_pub.publish(marker);

    }
    ROS_ERROR("Publish New Waypoint !!");  
    wp_pub.publish(wp_array_publish);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cdc_planner");
    CDCPlanner planner;
    planner.process();
    return 0;
}