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

typedef pcl::PointXYZ PointT;


using namespace std::chrono_literals;
using std::placeholders::_1;

double fx = 554.254691191187;  // Focal length in x-direction
double fy = 554.254691191187;  // Focal length in y-direction
double cx = 320.5;            // Principal point in x-direction
double cy = 240.5;            // Principal point in y-direction

class converter:public rclcpp::Node
{   
    public:
    converter() : Node("rgbd_pc_converter"), count_(0)
    {
        pc_publisher=this->create_publisher<sensor_msgs::msg::PointCloud2>("pc_data",1);
        depth_image_subscription = this->create_subscription<sensor_msgs::msg::Image>("/realsense/depth/image_raw", 10, std::bind(&converter::image_callback, this, _1));
    }
    
    private:
    void image_callback(const sensor_msgs::msg::Image &depth_img) const// (const std_msgs::msg::String & depth_img)
    {
        RCLCPP_INFO(rclcpp::get_logger("listener_callback_rgb"), "Receiving the rgb frame");
        RCLCPP_INFO(rclcpp::get_logger("listener_callback_rgb"), "Receiving the video frame");

        cv_bridge::CvImagePtr cv_ptr;
	    cv_ptr = cv_bridge::toCvCopy(depth_img, sensor_msgs::image_encodings::TYPE_32FC1);

        // Access the depth image as a cv::Mat
        cv::Mat depth_image = cv_ptr->image;

        // Get the number of rows and columns in the depth image
        int rows = depth_image.rows;
        int cols = depth_image.cols;

        pcl::PointCloud<pcl::PointXYZ> point_cloud;

        for (int row = 0; row < rows; ++row) {
            for (int col = 0; col < cols; ++col) {
                // Access the depth value at the current pixel
                float depth_value = depth_image.at<float>(row, col);

                // Check if the depth value is valid (not NaN)
                if (!std::isnan(depth_value)) {
                    // Calculate the 3D point coordinates in the camera frame
                    double x = (col - cx) * depth_value / fx;
                    double y = (row - cy) * depth_value / fy;
                    double z = depth_value;

                    // Create a PCL point and add it to the point cloud
                    pcl::PointXYZ point;
                    point.x = x;
                    point.y = y;
                    point.z = z;
                    point_cloud.push_back(point);
                }
            }
        }

        sensor_msgs::msg::PointCloud2 point_cloud_msg;
        pcl::toROSMsg(point_cloud, point_cloud_msg);
        point_cloud_msg.header.frame_id = "map";
        // ...

        point_cloud_msg.header.stamp = now();
        pc_publisher->publish(point_cloud_msg);         
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_subscription;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_publisher;
    size_t count_;
};

int main(int argc, char * argv[])
{

	//initialize ROS
	rclcpp::init(argc, argv);

	//create the 
	rclcpp::spin(std::make_shared<converter>());
	rclcpp::shutdown();
    return 0;
}