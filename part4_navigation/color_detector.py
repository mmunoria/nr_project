#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np
class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')
        # CV Bridge
        self.bridge = CvBridge()
        
        # Subscribers
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        
        # Publishers - publish red color intensity
        self.red_intensity_pub = self.create_publisher(Float32,'/red_color_intensity', 10)
        self.get_logger().info('Color Detector Started!')
        
    def image_callback(self, msg):
        """Process camera image to detect red color"""
        #self.get_logger().info('Image received!')
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
            
            # Convert BGR to HSV color space
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Define range for red color in HSV
            # Red has two ranges in HSV (wraps around 180)
            lower_red1 = np.array([0, 100, 100])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([160, 100, 100])
            upper_red2 = np.array([180, 255, 255])
            
            # Create masks for red color
            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask = cv2.bitwise_or(mask1, mask2)
            
            # Calculate percentage of red pixels
            total_pixels = cv_image.shape[0] * cv_image.shape[1]
            red_pixels = np.sum(mask > 0)
            red_percentage = (red_pixels / total_pixels) * 100.0
            
            # Publish red intensity
            intensity_msg = Float32()
            intensity_msg.data = red_percentage
            self.red_intensity_pub.publish(intensity_msg)
            if red_percentage > 5.0: # Log only if significantred detected
                self.get_logger().info(f'Red color detected:{red_percentage:.2f}%')
                
        except Exception as e:
            self.get_logger().error(f'Error processing image:{e}')
            
def main(args=None):
    rclpy.init(args=args)
    detector = ColorDetector()
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    detector.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
        main()