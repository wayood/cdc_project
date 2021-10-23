#include "middle_planner/middle_plan.h"

CDCPlanner::CDCPlanner(void)
{
    path_sub = nh.subscribe("/nav_path",1,&CDCPlanner::path_callback,this);
    lm_x_sub = nh.subscribe("/lm_position_x",1,&CDCPlanner::lm_x_callback,this);
    lm_y_sub = nh.subscribe("/lm_position_y",1,&CDCPlanner::lm_y_callback,this);
}

void CDCPlanner::path_callback(const nav_msgs::Path& msg)
{

    path = msg;
    ROS_INFO("sucess path");    
}

void CDCPlanner::lm_x_callback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    num = msg->data.size();
    
    for (int i = 0; i < num ; i++){
        lm_x[i] = msg->data[i];
    }
    // print all the remaining numbers
    // for(std::vector<float>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it)
    // {
    //     lm_x[i] = *it;
    //     i++;
    // }
    
    ROS_INFO("sucess lm_x");
    return;
    
}

void CDCPlanner::lm_y_callback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    num = msg->data.size();
    
    for (int i = 0; i < num ; i++){
        lm_y[i] = msg->data[i];
    }
    
    // print all the remaining numbers
    // for(std::vector<float>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it)
    // {
    //     lm_y[i] = *it;
    //     i++;
    // }
    ROS_INFO("sucess lm_y");
    return;
}

void CDCPlanner::process(void)
{
    ros::Rate loop_rate(10);
    int i = 0;
    Eigen::MatrixXf lm_sub_first = Eigen::MatrixXf::Ones(3,num);
    Eigen::MatrixXf wp;
    Eigen::MatrixXf lm_sub_current = Eigen::MatrixXf::Ones(3,num);
    // std::vector<std::vector<float>> lm_sub_first(3,std::vector<float>(100,1.0));
    while(ros::ok()){
        ROS_INFO("middle plan start");
        if(i == 0){
            prepare(lm_sub_first);
            
        }
        else{

            prepare(lm_sub_current);
            // Eigen::MatrixXf A = a_mat(lm_sub_first,lm_sub_current);
            // wp = CDC_plan(path);
            // CDC_publish(path,wp);
        }
        i++;
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
    ROS_INFO("lm count");
    return;
}

// Eigen::MatrixXf CDCPlanner::a_mat(Eigen::MatrixXf& lm_first,Eigen::MatrixXf& lm_current)
// {
//     Eigen::MatrixXf A;
//     Eigen::MatrixXf pinv = lm_first.completeOrthogonalDecomposition().pseudoInverse();
//     A = lm_current*pinv;
//     return A;
// }

// Eigen::MatrixXf CDCPlanner::CDC_plan(const nav_msgs::Path& path)
// {
//     Eigen::MatrixXf wp_matrix(3,sizeof(path.poses));
//     Eigen::MatrixXf new_wp_matrix(3,sizeof(path.poses));
//     for (int i = 0;i < sizeof(path.poses);i++){
//         wp_matrix(0,i) = path.poses[i].pose.position.x;
//         wp_matrix(1,i) = path.poses[i].pose.position.y;
//         wp_matrix(2,i) = 1;
//     }
//     // sizeof(path.poses)
//     new_wp_matrix = wp_matrix;
//     return new_wp_matrix;
// }

// void CDCPlanner::CDC_publish(const nav_msgs::Path& path,Eigen::MatrixXf wp)
// {
//     std_msgs::Header path_header ;
//     path_header = path.poses[0].header;
//     path_header.seq = 0;
//     path_header.stamp = ros::Time::now();
//     path_header.frame_id = "map";
//     middle_path.header = path_header;
//     Eigen::MatrixXf poses;
//     for (int i = 0;i< sizeof(path.poses);i++){
//         middle_path.poses[i].pose.position.x = wp(0,i); 
//         middle_path.poses[i].pose.position.y = wp(1,i); 
//         middle_path.poses[i].pose.position.z = path.poses[i].pose.position.z; 
//         middle_path.poses[i].pose.orientation.x = path.poses[i].pose.orientation.x;
//         middle_path.poses[i].pose.orientation.y = path.poses[i].pose.orientation.y;
//         middle_path.poses[i].pose.orientation.z = path.poses[i].pose.orientation.z;
//         middle_path.poses[i].pose.orientation.w = path.poses[i].pose.orientation.w;
//         middle_path.poses[i].header = path.poses[i].header;
//         middle_path.poses[i].header.seq = i;
//     }
// }

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cdc_planner");
    CDCPlanner planner;
    planner.process();
    return 0;
}