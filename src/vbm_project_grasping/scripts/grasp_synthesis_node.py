#!/usr/bin/env python3

import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from std_msgs.msg import Header
from sensor_msgs.msg import Image # Image is the message type
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
import numpy as np


class GraspSynthesis(Node):
    def __init__(self):
        super().__init__('grasp_points_systhesis')

        self.processed_pointcloud_subscriber = self.create_subscription(
            PointCloud2,
            '/processed_pointcloud_data',
            self.processed_pointcloud_cb,
            10
        )
        self.processed_pointcloud_subscriber
        self.pc_publisher= self.create_publisher(PointCloud2, "/thresholded_pointcloud", 10)

        self.threshold = 0.015

    def processed_pointcloud_cb(self, msg):

        processed_pointcloud = point_cloud2.read_points_numpy(msg)

        _coords = processed_pointcloud[processed_pointcloud[:, 2].argsort()] # Ascending order
        lower_limit = _coords[:, 2][-1] - self.threshold
        i = np.argwhere(_coords[:, 2] > lower_limit)

        thresh_coords = _coords[i].squeeze()

        
        # Flattened array
        flatten_coords = thresh_coords[:, :2]
        zeros_column = np.zeros((flatten_coords.shape[0], 1))
        modified_falttened_coords = np.hstack((flatten_coords, zeros_column))

        #################################################################################
        # # Save the flttened point cloud as a pcd file
        # point_cloud = o3d.geometry.PointCloud()
        # point_cloud.points = o3d.utility.Vector3dVector(modified_falttened_coords)

        # # Visualize the point cloud
        # o3d.io.write_point_cloud("hammer_flattened.pcd", point_cloud)
        #################################################################################


        fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]

        header = Header()
        header.frame_id = "world"
        header.stamp = self.get_clock().now().to_msg()
    
        pc2 = point_cloud2.create_cloud(header, fields, modified_falttened_coords)
        self.pc_publisher.publish(pc2)



def main(args=None):
    rclpy.init(args=args)
    image_subscriber = GraspSynthesis()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    
    rclpy.shutdown()
  
if __name__ == '__main__':
    main()