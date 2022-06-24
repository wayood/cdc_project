#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>


void callback(sensor_msgs::PointCloud2 pc2){
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_nan (new pcl::PointCloud<pcl::PointXYZ>); // NaN値あり
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>); // NaN値なし
  

  //sensor_msgs::PointCloud2からpcl::PointXYZに変換
  pcl::fromROSMsg(pc2, *cloud_nan);

  // NaN値が入ってるといろいろ面倒なので除去
  std::vector<int> nan_index;
  pcl::removeNaNFromPointCloud(*cloud_nan, *cloud, nan_index);

  // KD木を作っておく。近傍点探索とかが早くなる。
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ>);
  tree->setInputCloud(cloud);

  //近傍点探索に使うパラメータと結果が入る変数
  double radius = 0.1;  //半径r
  std::vector<int> k_indices;  //範囲内の点のインデックスが入る
  std::vector<float> k_sqr_distances;  //範囲内の点の距離が入る
  unsigned int max_nn = 1;  //何点見つかったら探索を打ち切るか。0にすると打ち切らない
  
  pcl::PointXYZ p;  //中心座標
  p.x = 0.5;
  p.y = 0.5;
  p.z = 0.0;

  //半径r以内にある点を探索
  tree->radiusSearch(p, radius, k_indices, k_sqr_distances, max_nn);
  
  if(k_indices.size() == 0) return;
  
  pcl::PointXYZ result = cloud->points[k_indices[0]];

  ROS_INFO("A nearest point of (0.5, 0.5) is...\nx: %lf, y:%lf, z:%lf", result.x, result.y, result.z);
  
}

int main(int argc, char** argv){
	ros::init(argc,argv,"z_at_xy");
	ros::NodeHandle nh;
	
	ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("points2", 5, callback);


	ros::spin();
	
	return 0;
}