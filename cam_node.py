#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from erics_cameras import RTSPCamera


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        
        # Create publisher
        image_qos = QoSProfile(depth=10)
        image_qos.reliability = ReliabilityPolicy.RELIABLE
        image_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        
        # Create publisher with QoS profile
        self.publisher = self.create_publisher(Image, 'camera/image_raw', image_qos)
        
        # Create cv_bridge instance
        self.bridge = CvBridge()
        
        self.cam = RTSPCamera()
        
        # Create timer for publishing at ~30 FPS
        timer_period = 1.0 / 30.0  # 30 FPS
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info('Camera publisher node started')
    
    def timer_callback(self):
        # Capture frame
        img = self.cam.take_image()
        
        if img is not None:
            try:
                # Convert OpenCV image to ROS Image message
                frame = img.get_array()
                gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                
                # Convert OpenCV image to ROS Image message
                ros_image = self.bridge.cv2_to_imgmsg(cv2.resize(gray_frame, (640, 480)), encoding='mono8')
                
                # Set timestamp
                ros_image.header.stamp = self.get_clock().now().to_msg()
                ros_image.header.frame_id = 'map'
                
                # Publish the image
                self.publisher.publish(ros_image)
                
            except Exception as e:
                self.get_logger().error(f'Error converting/publishing image: {e}')
        else:
            self.get_logger().warn('Failed to capture frame')
    
    def destroy_node(self):
        # Clean up resources
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        camera_publisher = CameraPublisher()
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        if 'camera_publisher' in locals():
            camera_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()