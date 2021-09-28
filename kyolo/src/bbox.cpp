#include "ros/ros.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/BoundingBox.h"
using namespace std;
void msgCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
    cout<<"Bouding Boxes (header):" << msg->header <<endl;
    cout<<"Bouding Boxes (image_header):" << msg->image_header <<endl;
    cout<<"Bouding Boxes (Class):" << msg->bounding_boxes[0].Class <<endl;
    cout<<"Bouding Boxes (xmin):" << msg->bounding_boxes[0].xmin <<endl;
    cout<<"Bouding Boxes (xmax):" << msg->bounding_boxes[0].xmax <<endl;
    cout<<"Bouding Boxes (ymin):" << msg->bounding_boxes[0].ymin <<endl;
    cout<<"Bouding Boxes (ymax):" << msg->bounding_boxes[0].ymax <<endl;
    cout << "\033[2J\033[1;1H";     // clear terminal
}
int main(int argc, char **argv)
{
    ros::init(argc,argv,"cood_subscriber");
    ros::NodeHandle nh;
    ros::Subscriber cood_sub = nh.subscribe("/darknet_ros/bounding_boxes",100,msgCallback);
    ros::spin();
    return 0;
}