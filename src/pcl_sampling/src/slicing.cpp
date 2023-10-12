#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

typedef pcl::PointXYZ PointT;

void visualizePointCloud(const pcl::PointCloud<PointT>::Ptr &cloud, const std::string &cloudName) {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<PointT>(cloud, cloudName);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudName);
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }
}

int main() {
    // Objects and data structures
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());

    // Load point cloud
    pcl::PCDReader reader;
    if (reader.read("/home/neha/VBM/vbm_project/pointcloud_hammer.pcd", *cloud) == -1) {
        PCL_ERROR("Couldn't read the file /home/neha/VBM/vbm_project/pointcloud_hammer.pcd \n");
        return (-1);
    }

    std::cout << "Size of the original point cloud: " << cloud->points.size() << std::endl;

    // Extract the point with the lowest z value
    PointT minPt, maxPt;
    pcl::getMinMax3D(*cloud, minPt, maxPt);
    std::cout << "Min x value: " << minPt.x << std::endl;
    std::cout << "Max x value: " << maxPt.x << std::endl;
    std::cout << "Min y value: " << minPt.y << std::endl;
    std::cout << "Max y value: " << maxPt.y << std::endl;
    std::cout << "Min Z value: " << minPt.z << std::endl;
    std::cout << "Max Z value: " << maxPt.z << std::endl;

    // Use a pass-through filter to retain only the points within a 5mm range above the highest point
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(minPt.z + 0.000008, minPt.z + 0.000009); 
    pass.filter(*cloud_filtered);

    std::cout << "Size of the filtered point cloud: " << cloud_filtered->points.size() << std::endl;

    std::vector<cv::Point2f> points2D;
    for (const auto& point : cloud_filtered->points) {
        points2D.push_back(cv::Point2f(point.x, point.y));
    }

    // Compute the convex hull using Delaunay triangulation
    std::vector<int> hullIndices;
    cv::convexHull(points2D, hullIndices);

    // Extract the rim points
    pcl::PointCloud<PointT>::Ptr rimPoints(new pcl::PointCloud<PointT>());
    rimPoints->points.reserve(hullIndices.size());
    for (int index : hullIndices) {
        rimPoints->points.push_back(cloud_filtered->points[index]);
    }
    rimPoints->width = rimPoints->points.size();
    rimPoints->height = 1;

    // Save the thresholded points to a new PCD file
    pcl::PCDWriter writer;
    writer.write("/home/neha/VBM/vbm_project/thresholded_pointcloud.pcd", *cloud_filtered);

    // Uncomment below if you wish to visualize 
    // visualizePointCloud(cloud_filtered, "Filtered PointCloud");
    // visualizePointCloud(rimPoints, "Rim Points");

    return 0;
}

