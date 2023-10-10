import rclpy
import open3d as o3d
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import read_points
from sensor_msgs.point_cloud2 import create_cloud_xyz32

class GraspAnalysisPlanning(Node):

    def __init__(self):

        super().__init__('grasp_analysis_planning')

        #TODO: Add a subscription to the topic produced by the c++ node that returns a point cloud image

    
    def process_image(self, data):

        pc_data = read_points(data, field_names=("x", "y", "z"), skip_nans=True)
        pc_list = []
        for data in pc_data:
            pc_list.append([data[0], data[1], data[2]])
        
        pc_numpy = np.asarray(pc_list)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pc_numpy)
        #TODO implement code for point cloud processing, distance thresholding, etc

    def main(args=None):
        # Initialize the rclpy library
        rclpy.init(args=args)

        # Create the node
        grasp_analysis_planning = GraspAnalysisPlanning()

        # Spin the node so the callback function is called.
        rclpy.spin(grasp_analysis_planning)

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        grasp_analysis_planning.destroy_node()

        # Shutdown the ROS client library for Python
        rclpy.shutdown()


if __name__ == '__main__':
    main()
