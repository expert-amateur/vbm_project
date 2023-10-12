import open3d as o3d
import numpy as np

def pcd_to_png(pcd_path, png_path):
    # Load the PCD file
    pcd = o3d.io.read_point_cloud(pcd_path)

    # Create a visualization
    vis = o3d.visualization.Visualizer()
    vis.create_window()

    # Add the point cloud geometry to the visualizer
    vis.add_geometry(pcd)

    # Show the visualization interactively
    vis.run()

    # Capture the visualization as an image
    vis.capture_screen_image(png_path)

    # Close the visualization window
    vis.destroy_window()


if __name__ == "__main__":
    pcd_file = "hammer_flattened.pcd"
    png_file = "hammer.png"
    pcd_to_png(pcd_file, png_file)
    print("sucessfully printed the file as png")
