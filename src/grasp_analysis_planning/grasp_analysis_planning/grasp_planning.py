#!/usr/bin/env python3
import rclpy
import open3d as o3d
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2 as pc2

import cv2 as cv

class GraspAnalysisPlanning(Node):

    def __init__(self):

        super().__init__('grasp_analysis_planning')

        # self.subscriber = self.create_subscription(PointCloud2, 'pc_data', self.process_image, 10)
        
        # self.subscriber



    def process_pcd(self):
        pcd = o3d.io.read_point_cloud("hammer_flattened.pcd")

        # Convert PointCloud to numpy array
        points = np.asarray(pcd.points)

        points -= points.min(axis=0)
        points /= points.ptp(axis=0)
        points *= 255.0
        points = points.astype(np.uint8)

        # Determine the size of the image
        max_x = int(np.max(points[:, 0]))
        max_y = int(np.max(points[:, 1]))
        min_x = int(np.min(points[:, 0]))
        min_y = int(np.min(points[:, 1]))

        # Create an image to hold the points
        image_size = (max_x - min_x + 1, max_y - min_y + 1)
        image = np.zeros(image_size, dtype=np.uint8)

        # Draw points on the image
        for point in points:
            x, y = int(point[0] - min_x), int(point[1] - min_y)
            image[y, x] = 255  # Set the pixel to white

        cv.imwrite("hammer_image.png", image)
    
    # def process_image(self, data):
# 
# 
# 
#     
        # gen = pc2.read_points(data, field_names=("x", "y"), skip_nans=True)

        # # Define the size of the image
        # width, height = data.width, data.height
# 
        # # Create an empty image
        # image = np.zeros((height, width), dtype=np.uint8)

        # # Loop through points and map them to the image
        # for x, y in gen:
        #     u = int(x)
        #     v = int(y)
            # 
        #     # Ensure the point coordinates are within the image dimensions
        #     if 0 <= u < width and 0 <= v < height:
        #         image[v, u] = 255  # set the pixel to white
        
        # image = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        # image = cv.GaussianBlur(image, (5 , 5), 0)
# 
        # edges = cv.Canny(image, 15, 21)
        # image = cv.merge([edges, edges, edges])
# 
        # cv.imshow('Image', image)
        #TODO implement code for point cloud processing, distance thresholding, etc

def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    grasp_analysis_planning = GraspAnalysisPlanning()

    # Spin the node so the callback function is called.
    # rclpy.spin(grasp_analysis_planning)

    grasp_analysis_planning.process_pcd()
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    grasp_analysis_planning.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
    main()
