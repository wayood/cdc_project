#include "middle_planner/middle_plan.h"

CDCPlanner::CDCPlanner(void)
    :path_subscribed(false),lm_x_subscribed(false),lm_y_subscribed(false)
{
    path_sub = nh.subscribe("/nav_path",1,&CDCPlanner::path_callback,this);
    lm_x_sub = nh.subscribe("/lm_position_x",1,&CDCPlanner::lm_x_callback,this);
    lm_y_sub = nh.subscribe("/lm_position_y",1,&CDCPlanner::lm_y_callback,this);
    path_pub = nh.advertise<nav_msgs::Path>("middle_path", 10);
}

void CDCPlanner::path_callback(const nav_msgs::Path& msg)
{
    path = msg;
    path_subscribed = true;
    ROS_INFO("sucess path");    
}

void CDCPlanner::lm_x_callback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    num = msg->data.size();
    
    for (int i = 0; i < num ; i++){
        lm_x[i] = msg->data[i];
    }
    lm_x_subscribed = true;
    ROS_INFO("sucess lm_x");
    return;
    
}

void CDCPlanner::lm_y_callback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    num = msg->data.size();
    
    for (int i = 0; i < num ; i++){
        lm_y[i] = msg->data[i];
    }
    lm_y_subscribed  = true;
    ROS_INFO("sucess lm_y");
    return;
}

void CDCPlanner::process(void)
{
    ros::Rate loop_rate(10);
    int i = 0;
    Eigen::MatrixXf wp;
    Eigen::MatrixXf lm_sub_first = Eigen::MatrixXf::Ones(3,100);
    while(ros::ok()){
        ROS_INFO("middle plan start");
        if(i == 0 && lm_x_subscribed && lm_y_subscribed)
        {
            lm_sub_first.conservativeResize(3,num);
            prepare(lm_sub_first);
            i++;

        }
        else if(lm_x[0] != 0 && lm_y[0] != 0 && i == 1 && path_subscribed)
        {
            Eigen::MatrixXf lm_sub_current = Eigen::MatrixXf::Ones(3,num);
            prepare(lm_sub_current);
            Eigen::MatrixXf A = a_mat(lm_sub_first,lm_sub_current);
            wp = CDC_plan(path,A);
            CDC_publish(path,wp);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void CDCPlanner::prepare(Eigen::MatrixXf& lm)
{
    for (int i = 0 ; i < lm.cols() ; i++){
        lm(0,i) = lm_x[i];
        lm(1,i) = lm_y[i];
    }
    return;
}

Eigen::MatrixXf CDCPlanner::a_mat(Eigen::MatrixXf& lm_first,Eigen::MatrixXf& lm_current)
{
    Eigen::MatrixXf A;
    Eigen::MatrixXf pinv = lm_first.completeOrthogonalDecomposition().pseudoInverse();
    A = lm_current * pinv;
    return A;
}

Eigen::MatrixXf CDCPlanner::CDC_plan(const nav_msgs::Path& path,Eigen::MatrixXf& A)
{
    Eigen::MatrixXf wp_matrix(3,sizeof(path.poses));
    Eigen::MatrixXf new_wp_matrix(3,sizeof(path.poses));
    for (int i = 0;i < sizeof(path.poses);i++){
        wp_matrix(0,i) = path.poses[i].pose.position.x;
        wp_matrix(1,i) = path.poses[i].pose.position.y;
        wp_matrix(2,i) = 1;
    }
    ROS_INFO("pseudo_INVERSE");
    new_wp_matrix = A * wp_matrix;
    return new_wp_matrix;
}

void CDCPlanner::CDC_publish(const nav_msgs::Path& path,Eigen::MatrixXf& wp)
{
    std_msgs::Header path_header;
    path_header = path.poses[0].header;
    path_header.seq = 0;
    path_header.stamp = ros::Time::now();
    path_header.frame_id = "map";
    middle_path.header = path_header;
    middle_path.poses = path.poses;
    for (int i = 0;i< sizeof(path.poses);i++){
        std::cout << middle_path.poses[i].pose.position.x << std::endl;
        middle_path.poses[i].pose.position.x = wp(0,i); 
        std::cout << middle_path.poses[i].pose.position.x << std::endl;
        middle_path.poses[i].pose.position.y = wp(1,i); 
        middle_path.poses[i].pose.position.z = path.poses[i].pose.position.z; 
        middle_path.poses[i].pose.orientation.x = path.poses[i].pose.orientation.x;
        middle_path.poses[i].pose.orientation.y = path.poses[i].pose.orientation.y;
        middle_path.poses[i].pose.orientation.z = path.poses[i].pose.orientation.z;
        middle_path.poses[i].pose.orientation.w = path.poses[i].pose.orientation.w;
        middle_path.poses[i].header = path.poses[i].header;
        middle_path.poses[i].header.seq = i;
    }
    path_pub.publish(middle_path);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cdc_planner");
    CDCPlanner planner;
    planner.process();
    return 0;
}