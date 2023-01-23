//2D implementation of the Ramer-Douglas-Peucker algorithm
//By Tim Sheerman-Chase, 2016
//Released under CC0
//https://en.wikipedia.org/wiki/Ramer%E2%80%93Douglas%E2%80%93Peucker_algorithm

#include "douglas_peucker.h"
using namespace std;

douglas_peucker::douglas_peucker(){
	sub_path = nh.subscribe("/nav_path",1,&douglas_peucker::path_callback,this);
	pub_wp = nh.advertise<waypoint_generator::Waypoint_array>("waypoint_", 1);
	pub_marker = nh.advertise<visualization_msgs::Marker>("waypoint_marker_new", 1);
}

douglas_peucker::~douglas_peucker(){
}

void douglas_peucker::path_callback(const nav_msgs::Path& msg)
{
	for(int i = 0;i < msg.poses.size();i++){
		Path_List.push_back(Point(msg.poses[i].pose.position.x, msg.poses[i].pose.position.y));
	}
    
    path_subscribed = true;
    ROS_INFO("sucess path");    
}

double douglas_peucker::PerpendicularDistance(const Point &pt, const Point &lineStart, const Point &lineEnd)
{
	double dx = lineEnd.first - lineStart.first;
	double dy = lineEnd.second - lineStart.second;

	//Normalise
	double mag = pow(pow(dx,2.0)+pow(dy,2.0),0.5);
	if(mag > 0.0)
	{
		dx /= mag; dy /= mag;
	}

	double pvx = pt.first - lineStart.first;
	double pvy = pt.second - lineStart.second;

	//Get dot product (project pv onto normalized direction)
	double pvdot = dx * pvx + dy * pvy;

	//Scale line direction vector
	double dsx = pvdot * dx;
	double dsy = pvdot * dy;

	//Subtract this from pv
	double ax = pvx - dsx;
	double ay = pvy - dsy;

	return pow(pow(ax,2.0)+pow(ay,2.0),0.5);
}

void douglas_peucker::RamerDouglasPeucker(const vector<Point> &pointList, double epsilon, vector<Point> &out)
{
	if(pointList.size()<2)
		throw invalid_argument("Not enough points to simplify");

	// Find the point with the maximum distance from line between start and end
	double dmax = 0.0;
	size_t index = 0;
	size_t end = pointList.size()-1;
	for(size_t i = 1; i < end; i++)
	{
		double d = PerpendicularDistance(pointList[i], pointList[0], pointList[end]);
		if (d > dmax)
		{
			index = i;
			dmax = d;
		}
	}

	// If max distance is greater than epsilon, recursively simplify
	if(dmax > epsilon)
	{
		// Recursive call
		vector<Point> recResults1;
		vector<Point> recResults2;
		vector<Point> firstLine(pointList.begin(), pointList.begin()+index+1);
		vector<Point> lastLine(pointList.begin()+index, pointList.end());
		RamerDouglasPeucker(firstLine, epsilon, recResults1);
		RamerDouglasPeucker(lastLine, epsilon, recResults2);
 
		// Build the result list
		out.assign(recResults1.begin(), recResults1.end()-1);
		out.insert(out.end(), recResults2.begin(), recResults2.end());
		if(out.size()<2)
			throw runtime_error("Problem assembling output");
	} 
	else 
	{
		//Just return start and end points
		out.clear();
		out.push_back(pointList[0]);
		out.push_back(pointList[end]);
	}
}

void douglas_peucker::process()
{
	
	while(false == ros::isShuttingDown()){
		ros::Rate loop_rate(100);
		wp_array.wp = {};
		if(path_subscribed){
			RamerDouglasPeucker(Path_List, 0.51,Waypoint_Out);
			for(size_t i=0;i< Waypoint_Out.size();i++)
			{
				wp.header.frame_id = "map";
				wp.header.stamp = ros::Time::now();
				wp.header.seq = i;
				wp.x = Waypoint_Out[i].first;
				wp.y = Waypoint_Out[i].second;
				wp_array.wp.push_back(wp);
				// cout << Waypoint_Out[i].first << "," << Waypoint_Out[i].second << endl;
			}
		}
		pub_wp.publish(wp_array);
		rviz_wp_publish();
		ros::spinOnce();
        loop_rate.sleep();
	}
}

void douglas_peucker::rviz_wp_publish(){

	for (int i = 0;i < wp_array.wp.size();i++){
		visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "waypoint_";
        marker.id = i;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration();

        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.2;
        marker.pose.position.x =  wp_array.wp[i].x;
        marker.pose.position.y = wp_array.wp[i].y;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 1;
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0f;
        pub_marker.publish(marker);
	}
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "douglas_peucker");
    douglas_peucker douglas_peucker;
    douglas_peucker.process();
    return 0;
}

// int main()
// {
// 	vector<Point> pointList;
// 	vector<Point> pointListOut;

// 	pointList.push_back(Point(0.0, 0.0));
// 	pointList.push_back(Point(1.0, 0.1));
// 	pointList.push_back(Point(2.0, -0.1));
// 	pointList.push_back(Point(3.0, 5.0));
// 	pointList.push_back(Point(4.0, 6.0));
// 	pointList.push_back(Point(5.0, 7.0));
// 	pointList.push_back(Point(6.0, 8.1));
// 	pointList.push_back(Point(7.0, 9.0));
// 	pointList.push_back(Point(8.0, 9.0));
// 	pointList.push_back(Point(9.0, 9.0));

// 	RamerDouglasPeucker(pointList, 0.51, pointListOut);

// 	cout << "result" << endl;
// 	for(size_t i=0;i< pointListOut.size();i++)
// 	{
// 		cout << pointListOut[i].first << "," << pointListOut[i].second << endl;
// 	}

// 	return 0;
// }

