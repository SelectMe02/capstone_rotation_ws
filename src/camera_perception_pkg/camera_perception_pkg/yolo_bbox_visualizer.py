#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from amr_msgs.msg import DetectionArray


class YoloBboxVisualizer(Node):
    def __init__(self):
        super().__init__('yolo_bbox_visualizer')

        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('detections_topic', '/detections')
        self.declare_parameter('annotated_topic', '/yolov8/annotated_image')
        self.declare_parameter('min_score', 0.3)

        self.image_topic = self.get_parameter('image_topic').value
        self.detections_topic = self.get_parameter('detections_topic').value
        self.annotated_topic = self.get_parameter('annotated_topic').value
        self.min_score = float(self.get_parameter('min_score').value)

        self.bridge = CvBridge()
        self.latest_detections = None

        image_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            image_qos
        )

        self.det_sub = self.create_subscription(
            DetectionArray,
            self.detections_topic,
            self.detections_callback,
            10
        )

        self.image_pub = self.create_publisher(
            Image,
            self.annotated_topic,
            10
        )

        self.get_logger().info(f'Subscribed image: {self.image_topic}')
        self.get_logger().info(f'Subscribed detections: {self.detections_topic}')
        self.get_logger().info(f'Publishing annotated image: {self.annotated_topic}')

    def detections_callback(self, msg):
        self.latest_detections = msg

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if self.latest_detections is not None:
            for det in self.latest_detections.detections:
                if det.score < self.min_score:
                    continue

                cx = int(det.bbox.center.position.x)
                cy = int(det.bbox.center.position.y)
                w = int(det.bbox.size.x)
                h = int(det.bbox.size.y)

                x1 = max(0, int(cx - w / 2))
                y1 = max(0, int(cy - h / 2))
                x2 = min(frame.shape[1] - 1, int(cx + w / 2))
                y2 = min(frame.shape[0] - 1, int(cy + h / 2))

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

                label = f'{det.class_name} {det.score:.2f}'
                cv2.putText(
                    frame,
                    label,
                    (x1, max(20, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2
                )

        out_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        out_msg.header = msg.header
        self.image_pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloBboxVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()