import numpy as np
import open3d as o3d

# Load the RGB image
rgb_image = np.load('path_to_npy_files')
# Load the depth image as a single-channel (grayscale) image
depth_image = np.load("path_to_npy_files")

# Intrinsic parameters 
# These are the paramters found by echoing the topic /realsense/camerainfo
fx = 554.254691191187  # Focal length in x-direction
fy = 554.254691191187 # Focal length in y-direction
cx = 320.5  # Principal point in x-direction
cy = 240.5  # Principal point in y-direction


# Create a point cloud from the depth image and RGB image
depth_scale = 1000.0  # Adjust this based on your depth image scale
depth = depth_image.astype(np.float32) / depth_scale
h, w = depth.shape
y, x = np.indices((h, w))
z = depth
x3d = (x - cx) * z / fx
y3d = (y - cy) * z / fy
xyz = np.dstack((x3d, y3d, z))

# Create an Open3D PointCloud
point_cloud = o3d.geometry.PointCloud()

# Assign the xyz coordinates to the point cloud
point_cloud.points = o3d.utility.Vector3dVector(xyz.reshape(-1, 3))

# Extract RGB colors from the RGB image
colors = rgb_image.reshape(-1, 3)

# Assign colors to the point cloud
point_cloud.colors = o3d.utility.Vector3dVector(colors / 255.0)

# Create an Open3D visualization window
o3d.visualization.draw_geometries([point_cloud])