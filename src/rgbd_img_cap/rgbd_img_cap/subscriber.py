import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import numpy as np
import os

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        self.subscription_rgb = self.create_subscription(
            Image,
            '/realsense/image_raw',
            self.listener_callback_rgb,
            10)
        self.subscription_rgb

        self.subscription_depthimage = self.create_subscription(
            Image,
            '/realsense/depth/image_raw',
            self.listener_callback_depthimage,
            10
        )
        self.subscription_depthimage
        self.test_imgs_path = "src/rgbd_img_cap/test_images"
        self.cur_dir = os.path.join(os.getcwd(), self.test_imgs_path)
        print(self.cur_dir)

    def listener_callback_rgb(self, data):
        self.get_logger().info('Receiving the rgb frame')
        output_filename = "rgb_img.npy"
        self.get_logger().info('Receiving the video frame')
        rgb_image = CvBridge().imgmsg_to_cv2(data)
        np.save(os.path.join(self.cur_dir, output_filename), rgb_image)


    def listener_callback_depthimage(self, data):
        self.get_logger().info('Receiving the depth frame')
        output_filename = "depth_img.npy"
        depth_image = CvBridge().imgmsg_to_cv2(data)

        # Save the image in numpy array in float32 for dense point cloud
        depth_array = np.array(depth_image, dtype=np.float32)

        # Save the depth image in numpy array since 
        # cv2.imwrite saves the image in corresponding int8 integers
        np.save(os.path.join(self.cur_dir, output_filename), depth_array)
        


def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    
    rclpy.shutdown()
  
if __name__ == '__main__':
  main()