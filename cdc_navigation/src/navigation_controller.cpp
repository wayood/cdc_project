#include "CDC_navigation/navigation_controller.h"

Navigation::Navigation(void)
:wp_flag(false),wp_new_flag(false)
{
    wp_init_sub_ = nh.subscribe("/waypoint",1,&Navigation::wp_callback,this);
    wp_new_sub_ = nh.subscribe("/waypoint_new",1,&Navigation::wp_new_callback,this);
    odom_sub_ = nh.subscribe("/odom",1,&Navigation::odom_callback,this);
    goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
}

// ,wp_new_flag(false)
void Navigation::wp_callback(const waypoint_generator::Waypoint_array& msg)
{
    wp_array_subscribe = msg;
    wp_flag = true;
}

void Navigation::odom_callback(const nav_msgs::OdometryConstPtr& msg)
{
    odom_subscribe = msg;
}

void Navigation::wp_new_callback(const waypoint_generator::Waypoint_array& msg)
{
    wp_new_array_subscribe = msg;
    wp_new_flag = true;
}

void Navigation::process(void)
{
    ros::Rate loop_rate(50);
    int count = 0;
    float Tolerence = 0.5;
    ROS_INFO("==================================");
    ROS_INFO("|       Navigation start !!      |");
    ROS_INFO("==================================");
    while(ros::ok()){
        if(wp_flag && wp_array_subscribe.wp.size() > 0){
            if (count == 0){  
                nextGoal(wp_array_subscribe,count);
            }
            if(wp_new_flag){
                nextGoal(wp_new_array_subscribe,count);
            }
                
            double l = sqrt(pow(wp_array_subscribe.wp[count].x - odom_subscribe->pose.pose.position.x,2) + pow(wp_array_subscribe.wp[count].y - odom_subscribe->pose.pose.position.y,2));
            if (l < Tolerence){
                count += 1;
                if(wp_array_subscribe.wp.size() < count){
                    ROS_INFO("Finish Operation");
                    break;
                }
                nextGoal(wp_new_array_subscribe,count);
                ROS_INFO("Next Goal");
            }
            
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}


void Navigation::nextGoal(const waypoint_generator::Waypoint_array& wp_array_subscribe,int count){
    geometry_msgs::PoseStamped goal_point;

    goal_point.pose.position.x = wp_array_subscribe.wp[count].x;
    goal_point.pose.position.y = wp_array_subscribe.wp[count].y;
    goal_point.pose.position.z =  0;
    goal_point.pose.orientation.x = 0;
    goal_point.pose.orientation.y = 0;
    goal_point.pose.orientation.z = 0;
    goal_point.pose.orientation.w = 1.0;
    goal_point.header.stamp = ros::Time::now();
    goal_point.header.frame_id = "map";
    goal_point.header.seq = count ;
    goal_pub_.publish(goal_point);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "navigation_controller");
    Navigation navi;
    navi.process();
    return 0;
}