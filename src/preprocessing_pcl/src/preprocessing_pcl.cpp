#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <boost/make_shared.hpp>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

using std::placeholders::_1;

typedef pcl::PointXYZ PointT;

void visualizePointCloud(const pcl::PointCloud<PointT>::Ptr &cloud, const std::string &cloudName) {
    // Create a viewer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    // Add our cloud to the viewer
    viewer->addPointCloud<PointT>(cloud, cloudName);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudName);
    
    // Keep viewer open until user closes it
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }
}

class preprocessor: public rclcpp::Node
{
    public:
    preprocessor() : Node("preprocessing_pcl")
    {
        preprocessed_pc_publisher=this->create_publisher<sensor_msgs::msg::PointCloud2>("pc_preprocessed_data",1);
        raw_pc_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>("pc_data", 10, std::bind(&preprocessor::pcl_preprocessor_callback, this, _1));
    }
    private:
    void pcl_preprocessor_callback(const sensor_msgs::msg::PointCloud2 &raw_pc) const// (const std_msgs::msg::String & depth_img)
    {   
        // PCL still uses boost::shared_ptr internally
        //boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud =boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud=std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        // Convert the boost::shared_ptr to std::shared_ptr
        //std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud_std = cloud_boost;

        // This will convert the message into a pcl::PointCloud
        pcl::fromROSMsg(raw_pc, *cloud);

        std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;
        std::shared_ptr<pcl::PCLPointCloud2> cloud_pcl2=std::make_shared<pcl::PCLPointCloud2>();
        pcl::toPCLPointCloud2(*cloud, *cloud_pcl2);
        // Create the filtering object
        pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
        sor.setInputCloud (cloud_pcl2);
        sor.setLeafSize (0.00001f, 0.00001f, 0.00001f);
        std::shared_ptr<pcl::PCLPointCloud2> cloud_filtered;
        sor.filter (*cloud_filtered);

        std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height<< " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;

        // All the objects needed
        pcl::PCDReader reader;
        pcl::PassThrough<PointT> pass;
        pcl::NormalEstimation<PointT, pcl::Normal> ne;
        pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
        pcl::PCDWriter writer;
        pcl::ExtractIndices<PointT> extract;
        pcl::ExtractIndices<pcl::Normal> extract_normals;
        pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

        // Datasets
        pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
        pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
        pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);



    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr raw_pc_subscription;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr preprocessed_pc_publisher;
    
};

int
main ()
{
  // All the objects needed
  pcl::PCDReader reader;
  pcl::PassThrough<PointT> pass;
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
  pcl::PCDWriter writer;
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

  // Datasets
  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);

  // Read in the cloud data
  reader.read ("/home/neha/Downloads/pointcloud_copy1.pcd", *cloud);
  std::cerr << "PointCloud has: " << cloud->size () << " data points." << std::endl;

  // Build a passthrough filter to remove spurious NaNs and scene background
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, 1.5);
  pass.filter (*cloud_filtered);
  std::cerr << "PointCloud after filtering has: " << cloud_filtered->size () << " data points." << std::endl;

  // Estimate point normals
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud_filtered);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);

  // Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight (0.1);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.03);
  seg.setInputCloud (cloud_filtered);
  seg.setInputNormals (cloud_normals);
  // Obtain the plane inliers and coefficients
  seg.segment (*inliers_plane, *coefficients_plane);
  std::cout << "Plane coefficients: " << *coefficients_plane << std::endl;

//   // Extract the planar inliers from the input cloud
//   extract.setInputCloud (cloud_filtered);
//   extract.setIndices (inliers_plane);
//   extract.setNegative (false);

//   // Write the planar inliers to disk
//   pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
//   extract.filter (*cloud_plane);
//   std::cerr << "PointCloud representing the planar component: " << cloud_plane->size () << " data points." << std::endl;
//   writer.write ("/home/neha/Downloads/pointcloud_copy1.pcd", *cloud_plane, false);
  
// //   visualizePointCloud(cloud_plane, "plane");
//   // Remove the planar inliers, extract the rest
//   extract.setNegative (true);
//   extract.filter (*cloud_filtered2);
//   extract_normals.setNegative (true);
//   extract_normals.setInputCloud (cloud_normals);
//   extract_normals.setIndices (inliers_plane);
//   extract_normals.filter (*cloud_normals2);

//   // Create the segmentation object for cylinder segmentation and set all the parameters
//   seg.setOptimizeCoefficients (true);
//   seg.setModelType (pcl::SACMODEL_CYLINDER);
//   seg.setMethodType (pcl::SAC_RANSAC);
//   seg.setNormalDistanceWeight (0.1);
//   seg.setMaxIterations (10000);
//   seg.setDistanceThreshold (0.05);
//   seg.setRadiusLimits (0, 0.1);
//   seg.setInputCloud (cloud_filtered2);
//   seg.setInputNormals (cloud_normals2);

//   // Obtain the cylinder inliers and coefficients
//   seg.segment (*inliers_cylinder, *coefficients_cylinder);
//   std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

//   // Write the cylinder inliers to disk
//   extract.setInputCloud (cloud_filtered2);
//   extract.setIndices (inliers_cylinder);
//   extract.setNegative (false);
//   pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
//   extract.filter (*cloud_cylinder);

//   if (cloud_cylinder->points.empty ()) 
//     std::cerr << "Can't find the cylindrical component." << std::endl;
//   else
//   {
// 	  std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->size () << " data points." << std::endl;
// 	  writer.write ("/home/neha/Downloads/pointcloud_copy1.pcd", *cloud_cylinder, false);
//     //   visualizePointCloud(cloud_cylinder, "cylinder");
//   }
  return (0);
}


