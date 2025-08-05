#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
from erics_cameras import RTSPCamera
import numpy as np


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        
        # Create publisher
        image_qos = QoSProfile(depth=10)
        image_qos.reliability = ReliabilityPolicy.RELIABLE
        image_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        
        # Create publisher with QoS profile
        self.publisher = self.create_publisher(Image, '/image_rect', image_qos)
        self.info_publisher = self.create_publisher(CameraInfo, '/camera_info_rect', image_qos)
        
        # Create cv_bridge instance
        self.bridge = CvBridge()
        
        self.cam = RTSPCamera()

        self.cam_mat = np.array([[1028,0,609.03],[0,1028.4,397.99],[0,0,1]])
        self.dist_coeffs = np.array([[-0.35952,0.080321,0.001794,-0.001439,0.025185]])

        self.undistorted_cam_mat, _ = cv2.getOptimalNewCameraMatrix(self.cam_mat, self.dist_coeffs, (1920, 1080), 0, (1920, 1080))
        self.m1, self.m2 = cv2.initUndistortRectifyMap(self.cam_mat, self.dist_coeffs, None, self.undistorted_cam_mat, (1920, 1080), cv2.CV_16SC2)
        
        # Create timer for publishing at ~30 FPS
        timer_period = 1.0 / 30.0  # 30 FPS
        self.timer = self.create_timer(timer_period, self.timer_callback)

        info_timer_period = 1.0
        self.info_timer = self.create_timer(info_timer_period, self.info_timer_callback)

        self.camera_info = CameraInfo()
        self.camera_info.header.stamp = self.get_clock().now().to_msg()
        self.camera_info.header.frame_id = 'map'
        self.camera_info.height = 1080
        self.camera_info.width = 1920
        self.camera_info.distortion_model = "pinhole"
        self.camera_info.k = [1028.0, 0.0, 609.03, 0.0, 1028.4, 397.99, 0.0, 0.0, 1.0]
        self.camera_info.d = []
        self.camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.camera_info.p = [1028.0, 0.0, 609.03, 0.0, 0.0, 1028.4, 397.99, 0.0, 0.0, 0.0, 1.0, 0.0]
        
        self.get_logger().info('Camera publisher node started')
    
    def timer_callback(self):
        # Capture frame
        img = self.cam.take_image()
        
        if img is not None:
            try:
                # Convert OpenCV image to ROS Image message
                frame = img.get_array()
                # gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                # ros_image = self.bridge.cv2_to_imgmsg(cv2.resize(gray_frame, (640, 480)), encoding='mono8')
                frame_undistorted = cv2.remap(frame, self.m1, self.m2, cv2.INTER_LINEAR)
                
                # Convert OpenCV image to ROS Image message
                ros_image = self.bridge.cv2_to_imgmsg(frame_undistorted, encoding='8UC3')
                
                # Set timestamp
                ros_image.header.stamp = self.get_clock().now().to_msg()
                ros_image.header.frame_id = 'map'
                
                # Publish the image
                self.publisher.publish(ros_image)
                
            except Exception as e:
                self.get_logger().error(f'Error converting/publishing image: {e}')
        else:
            self.get_logger().warn('Failed to capture frame')
    
    def info_timer_callback(self):
        self.info_publisher.publish(self.camera_info)

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