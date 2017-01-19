#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <boost/foreach.hpp>

ros::Publisher pub;

  // All the objects needed
//  pcl::PCDReader reader;
//  pcl::NormalEstimation<PointT, pcl::Normal> ne;
//  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
//  pcl::PCDWriter writer;
//  pcl::ExtractIndices<PointT> extract;
//  pcl::ExtractIndices<pcl::Normal> extract_normals;
//  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

  // Datasets
//  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
//  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
//  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
//  pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
//  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
//  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
//  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);

void zed_pointcloud_callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& zed_point_cloud)
{
  ROS_INFO("Point Cloud Received With %d Points", zed_point_cloud->points.size());

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filteredz (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filteredx (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filteredy (new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::PassThrough<pcl::PointXYZRGB> pass;
 
  pass.setInputCloud (zed_point_cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-0.5, 0.5);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filteredz);

  pass.setInputCloud (cloud_filteredz);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (0, 15);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filteredx);

  ROS_INFO("Point Cloud After Filter %d Points", cloud_filteredx->points.size());

  pub.publish (cloud_filteredx);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "zed_pcl_segmenter");
  ros::NodeHandle nh;
  pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("zed_filtered_pointcloud", 1); 
  ros::Subscriber sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB> >("/zed/point_cloud/cloud_registered", 1, zed_pointcloud_callback);
  ros::spin();
}
