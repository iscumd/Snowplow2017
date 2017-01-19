#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>

void zed_pointcloud_callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& zed_point_cloud)
{
  ROS_INFO("Point Cloud Received");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "zed_pcl_segmenter");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB> >("/zed/point_cloud/cloud_registered", 1, zed_pointcloud_callback);
  ros::spin();
}
