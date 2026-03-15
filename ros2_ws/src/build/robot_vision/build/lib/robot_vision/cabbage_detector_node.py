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

        self.model  = YOLO('/home/jorjeen/plant/cabbage_project/cab_model/best.pt')
        self.bridge = CvBridge()

        self.CAMERA_HEIGHT = 32.5
        self.FOCAL_LENGTH  = 440

        self.last_cmd     = ""
        self.frame_width  = 640
        self.frame_height = 480

        self._triggered = False

        # -------- Subscribers --------
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

        # -------- Publishers --------
        self.pub = self.create_publisher(
            Image,
            '/camera2/image_detected',   
            10)

        # Format: "size_cm"  เช่น "12.4"
        # ถ้าไม่พบ: "0.0"
        self.detect_result_pub = self.create_publisher(
            String,
            '/quin/detect_result',
            10)

        self.get_logger().info('Cabbage Detector started — waiting for /quin/detect_trigger...')

    # ==================================================
    # Trigger
    # ==================================================

    def detect_trigger_callback(self, msg: Bool):
        if not msg.data:
            return
        if self._triggered:
            self.get_logger().warn("Already scanning — duplicate trigger ignored")
            return
        self._triggered = True
        self.get_logger().info("Detection triggered — scanning next frame...")

    # ==================================================
    # Helpers
    # ==================================================

    def send_command(self, cmd):
        if cmd != self.last_cmd:
            print(f"CMD: {cmd}")
            self.last_cmd = cmd

    def estimate_size(self, pixel_width):
        return (pixel_width * self.CAMERA_HEIGHT) / self.FOCAL_LENGTH

    # ==================================================
    # Image Callback
    # ==================================================

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.frame_height, self.frame_width = frame.shape[:2]

        # ไม่ถูก trigger — publish raw frame ผ่านไปเลย
        if not self._triggered:
            out_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.pub.publish(out_msg)
            return

        # ---- YOLO รันเฉพาะตอน triggered ----
        frame_small = cv2.resize(frame, (320, 320))
        scale_x = frame.shape[1] / 320
        scale_y = frame.shape[0] / 320

        results = self.model(
            frame_small,
            conf=0.5,
            imgsz=320,
            device="cpu",
            verbose=False
        )[0]

        top_bound    = self.frame_height * 0.15
        bottom_bound = self.frame_height * 0.5

        cv2.line(frame, (0, int(top_bound)),
                 (self.frame_width, int(top_bound)),    (255, 0, 0), 1)
        cv2.line(frame, (0, int(bottom_bound)),
                 (self.frame_width, int(bottom_bound)), (255, 0, 0), 1)

        if len(results.boxes) == 0:
            # ไม่พบกะหล่ำ
            self.send_command("MOVE_FORWARD")
            cv2.putText(frame, "MOVE FORWARD", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), 2)
            cv2.putText(frame, "ไม่พบกะหล่ำปลี", (10, 65),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

            self._publish_result(size_cm=0.0)
            self._triggered = False

        else:
            # เลือก box ที่ใหญ่สุด
            best_box = max(results.boxes,
                           key=lambda b: (b.xyxy[0][2] - b.xyxy[0][0]) *
                                         (b.xyxy[0][3] - b.xyxy[0][1]))

            x1, y1, x2, y2 = best_box.xyxy[0]
            x1 = int(x1 * scale_x)
            x2 = int(x2 * scale_x)
            y1 = int(y1 * scale_y)
            y2 = int(y2 * scale_y)

            center_y    = (y1 + y2) / 2
            pixel_width = x2 - x1
            size_cm     = self.estimate_size(pixel_width)

            # วาด bbox + ขนาด
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"{size_cm:.1f} cm", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            if top_bound <= center_y <= bottom_bound:
                # อยู่ใน zone — รายงานขนาด
                self.send_command("STOP")
                self.get_logger().info(f"Cabbage detected — size: {size_cm:.1f} cm")

                cv2.putText(frame, f"ขนาด: {size_cm:.1f} cm", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)

                self._publish_result(size_cm=size_cm)
                self._triggered = False

            else:
                # ยังไม่อยู่ใน zone
                self.send_command("MOVE_FORWARD")
                cv2.putText(frame, "MOVE FORWARD", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), 2)
                cv2.putText(frame, f"ขนาด: {size_cm:.1f} cm", (10, 65),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

        out_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.pub.publish(out_msg)

    # ==================================================
    # Publish Result
    # ==================================================

    def _publish_result(self, size_cm: float):
        """
        ส่งขนาดกะหล่ำให้ mission_node
        Format: "size_cm"  เช่น "12.4"
        ถ้าไม่พบ: "0.0"
        """
        msg      = String()
        msg.data = f"{size_cm:.1f}"
        self.detect_result_pub.publish(msg)
        self.get_logger().info(f"Result published → {msg.data} cm")


def main(args=None):
    rclpy.init(args=args)
    node = CabbageDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()