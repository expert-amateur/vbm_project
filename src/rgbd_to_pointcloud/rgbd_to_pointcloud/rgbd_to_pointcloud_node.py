import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
import numpy as np
import open3d as o3d

class RGBDToPointCloudNode(Node):
    def __init__(self):
        super().__init__('rgbd_to_pointcloud_node')
        self.subscription = self.create_subscription(
            Image,
            '/realsense/depth/image_raw',
            self.listener_callback,
            10
        )
        self.publisher = self.create_publisher(PointCloud2, '/pointcloud_output', 10)

    def listener_callback(self, msg):
        # Intrinsic parameters (from your provided code)
        fx, fy = 554.254691191187, 554.254691191187
        cx, cy = 320.5, 240.5
        
        # Convert the ROS Image message to a numpy array and reshape it
        depth_image_np = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        
        # Following the logic from your provided code to generate the point cloud
        depth_scale = 1000.0
        depth = depth_image_np.astype(np.float32) / depth_scale
        h, w = depth.shape
        y, x = np.indices((h, w))
        z = depth
        x3d = (x - cx) * z / fx
        y3d = (y - cy) * z / fy
        xyz = np.dstack((x3d, y3d, z))

        # Create an Open3D PointCloud
        point_cloud_o3d = o3d.geometry.PointCloud()
        point_cloud_o3d.points = o3d.utility.Vector3dVector(xyz.reshape(-1, 3))

        # Convert Open3D point cloud to ROS PointCloud2 message
        pointcloud_msg = open3d_to_ros(point_cloud_o3d)
        self.publisher.publish(pointcloud_msg)

def open3d_to_ros(open3d_cloud):
    ros_cloud = PointCloud2()
    ros_cloud.header.frame_id = 'camera_link'
    ros_cloud.header.stamp = rclpy.time.Time().to_msg()

    # Convert Open3D point cloud to XYZ data
    xyz = np.asarray(open3d_cloud.points)

    # Define a constant color (e.g., blue)
    blue_color = np.array([255, 0, 0], dtype=np.uint8)
    rgb_color = np.dot(blue_color, [1, 256, 65536])  # Encode RGB color as a single uint32 number

    # Create a RGB data array filled with the same color
    rgb_data = np.full((len(xyz), 1), rgb_color, dtype=np.uint32)

    # Stack XYZ and RGB data
    data = np.hstack((xyz, rgb_data))

    data = data.reshape(-1)  # Flatten the data array

    # Define the fields of the PointCloud2 message
    ros_cloud.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)
    ]

    ros_cloud.point_step = 16  # Each point has 'x', 'y', 'z', and 'rgb'
    ros_cloud.row_step = ros_cloud.point_step * len(data)
    ros_cloud.is_dense = True
    ros_cloud.is_bigendian = False
    ros_cloud.width = len(xyz)
    ros_cloud.height = 1
    ros_cloud.data = data.tobytes()

    return ros_cloud

def main(args=None):
    rclpy.init(args=args)
    rgbd_to_pointcloud_node = RGBDToPointCloudNode()
    rclpy.spin(rgbd_to_pointcloud_node)
    rgbd_to_pointcloud_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

