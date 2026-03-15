#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2


class CabbageDetector(Node):

    def __init__(self):
        super().__init__('cabbage_detector')

        # YOLO model
        self.model = YOLO('/home/jorjeen/plant/cabbage_project/cab_model/best.pt')

        self.bridge = CvBridge()

        # camera calibration
        self.CAMERA_HEIGHT = 32.5
        self.FOCAL_LENGTH  = 440

        self._triggered = False

        # subscribers
        self.create_subscription(
            Image,
            '/camera2/image_raw',
            self.image_callback,
            1)

        self.create_subscription(
            Bool,
            '/quin/detect_trigger',
            self.detect_trigger_callback,
            10)

        # publishers
        self.image_pub = self.create_publisher(
            Image,
            '/camera2/image_detected',
            10)

        self.result_pub = self.create_publisher(
            String,
            '/quin/detect_result',
            10)

        self.get_logger().info("Cabbage detector ready")

    # -------------------------------------------------

    def detect_trigger_callback(self, msg):

        if msg.data:
            self._triggered = True
            self.get_logger().info("Detection triggered")

    # -------------------------------------------------

    def estimate_size(self, pixel_width):

        size_cm = (pixel_width * self.CAMERA_HEIGHT) / self.FOCAL_LENGTH

        return size_cm

    # -------------------------------------------------

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # ถ้ายังไม่ trigger → แค่ส่งภาพผ่าน
        if not self._triggered:

            out = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.image_pub.publish(out)
            return

        # run YOLO
        results = self.model(frame, conf=0.5, imgsz=640, device="cpu")[0]

        size_cm = 0.0

        if len(results.boxes) > 0:

            # เอา detection แรก
            box = results.boxes[0]

            x1, y1, x2, y2 = box.xyxy[0]

            x1 = int(x1)
            x2 = int(x2)
            y1 = int(y1)
            y2 = int(y2)

            pixel_width = x2 - x1

            size_cm = self.estimate_size(pixel_width)

            cv2.rectangle(frame,(x1,y1),(x2,y2),(0,255,0),2)

            cv2.putText(
                frame,
                f"{size_cm:.1f} cm",
                (x1, y1-10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0,255,0),
                2
            )

            self.get_logger().info(f"Cabbage size: {size_cm:.1f} cm")

        else:

            self.get_logger().warn("No cabbage detected")

        # publish result ให้ mission node
        msg_out = String()

        if size_cm > 0:
            msg_out.data = f"{size_cm:.1f},1"
        else:
            msg_out.data = "0,-1"

        self.result_pub.publish(msg_out)

        self.get_logger().info(f"Published result: {msg_out.data}")

        self._triggered = False

        # publish image
        out = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.image_pub.publish(out)


def main(args=None):

    rclpy.init(args=args)

    node = CabbageDetector()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()