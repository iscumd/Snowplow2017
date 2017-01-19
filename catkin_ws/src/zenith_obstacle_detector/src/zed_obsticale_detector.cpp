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

ros::Publisher filter_pub, cylinder_pub, plane_pub;

  // All the objects needed
//  pcl::PCDReader reader;
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg; 
//  pcl::PCDWriter writer;
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
//  pcl::ExtractIndices<pcl::Normal> extract_normals;

  // Datasets
//  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
//  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
//  pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
//  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);

void zed_pointcloud_callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& zed_point_cloud)
{
  ROS_INFO("Point Cloud Received With %d Points", zed_point_cloud->points.size());

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filteredz (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filteredx (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filteredy (new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
 
  pass.setInputCloud (zed_point_cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-0.5, 0.5);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filteredz);

  pass.setInputCloud (cloud_filteredz);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (0, 4);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filteredy);


  pass.setInputCloud (cloud_filteredy);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (0, 4);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filteredx);

  ROS_INFO("Point Cloud After Filter %d Points", cloud_filteredx->points.size());

  filter_pub.publish (cloud_filteredx);

  ROS_INFO("Published Point Cloud");

    // Estimate point normals
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud_filteredx);
  ne.setKSearch (10);
  ne.compute (*cloud_normals);
  ROS_INFO("Norms Computed");

  // Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight (0.1);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.03);
  seg.setInputCloud (cloud_filteredx);
  seg.setInputNormals (cloud_normals);
  // Obtain the plane inliers and coefficients
  seg.segment (*inliers_plane, *coefficients_plane);
//  std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;
  ROS_INFO("Planner Segmentation Complete");
  // Extract the planar inliers from the input cloud
  extract.setInputCloud (cloud_filteredx);
  extract.setIndices (inliers_plane);
  extract.setNegative (false);

  // Write the planar inliers to disk
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
  extract.filter (*cloud_plane);
  ROS_INFO("Planner Extraction Complete");
//  std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
//  writer.write ("table_scene_mug_stereo_textured_plane.pcd", *cloud_plane, false);

  plane_pub.publish (cloud_plane);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "zed_pcl_segmenter");
  ros::NodeHandle nh;
  filter_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("zed_filtered_pointcloud", 1); 
  cylinder_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("zed_cylinder_pointcloud", 1);
  plane_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("zed_plane_pointcloud", 1);
  ros::Subscriber sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB> >("/zed/point_cloud/cloud_registered", 1, zed_pointcloud_callback);
  ros::spin();
}
