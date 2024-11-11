#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, PointCloud2

import cv2 as cv
from cv_bridge import CvBridge
from ultralytics import YOLO

from ament_index_python.packages import get_package_share_directory


class SemanticMapping(Node):

    def __init__(self):
        super().__init__("semantic_mapping")
        self.image_sub = self.create_subscription(
            Image, "rtabmap_image", self.image_callback, 10
        )
        self.point_cloud_sub = self.create_subscription(
            PointCloud2, "rtabmap_point_cloud", self.point_cloud_callback, 10
        )
        # self.timer = self.create_timer(0.5, self.timer_callback)

        self.yolo_image = None
        self.point_cloud = None
        self.model = YOLO(
            get_package_share_directory("orb_slam3_ros2")
            + "/models/yolov8n.pt"
        )

        self.bridge = CvBridge()

    def image_callback(self, msg):
        self.get_logger().info("Image received")
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        results = self.model(self.bridge.imgmsg_to_cv2(msg))
        bboxes = []
        for result in results:
            for box in result.boxes:
                # Get the confidence value
                confidence = box.conf.item()

                # Check if the confidence value is at least 0.8
                if confidence >= 0.5:
                    bboxes.append(box)
        #             # Get bounding box coordinates in top, left, bottom, right
        #             # bbox = box.xyxy[0].to("cpu").detach().numpy().copy()
        #             bbox = box.xyxy[0].cpu().numpy()
        #
        #             bboxes.append(bbox)
        #
        #             # Calculate the center of the bounding box
        #             bbox_center = [
        #                 int((bbox[0] + bbox[2]) / 2),
        #                 int((bbox[1] + bbox[3]) / 2),
        #             ]
        #
        #             # Draw a circle at the center of the bounding box
        #             cv.circle(
        #                 image,
        #                 (bbox_center[0], bbox_center[1]),
        #                 radius=5,
        #                 color=(0, 0, 255),
        #                 thickness=-1,
        #             )

        for box in bboxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            self.get_logger().info(f"x1: {x1}, y1: {y1}, x2: {x2}, y2: {y2}")
            cv.rectangle(
                image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2
            )

        # self.get_logger().info(f"results: {results}")
        # annotated_frame = results[0].plot()
        cv.imshow("grayscale_image", image)
        cv.waitKey(1)

    def point_cloud_callback(self, msg):
        self.point_cloud = msg


def main(args=None):
    rclpy.init(args=args)
    semantic_mapping = SemanticMapping()
    rclpy.spin(semantic_mapping)
    semantic_mapping.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
