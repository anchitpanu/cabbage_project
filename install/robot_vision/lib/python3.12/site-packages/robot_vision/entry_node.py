import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool

from cv_bridge import CvBridge
import torch
from ultralytics import YOLO


class EntryNode(Node):

    def __init__(self):
        super().__init__('entry_node')

        # ---------------- PARAMETERS ----------------
        self.MODEL_PATH = '/home/jorjeen/plant/cabbage_project/entering_model/enterbest.pt'

        self.CONF_THRESH = 0.4
        self.IMG_SIZE = 320           # smaller = faster inference
        self.TARGET_CLASS = 0

        self.TORCH_THREADS = 2        # good for Raspberry Pi
        self.FRAME_SKIP = 6           # process every 3rd frame

        # movement parameters
        self.TARGET_X = 530.0
        self.ALIGN_TOL = 25

        self.FORWARD_SPEED = 0.25
        self.TURN_SPEED = 0.35
        self.SEARCH_SPEED = 0.25

        # ---------------- TOOLS ----------------
        self.bridge = CvBridge()
        self.frame_count = 0

        # ---------------- PUBLISHERS ----------------
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.center_pub = self.create_publisher(
            Float32,
            '/tag_center_x',
            10
        )

        self.detect_pub = self.create_publisher(
            Bool,
            '/vision/detection_active',
            10
        )

        self.image_pub = self.create_publisher(
            Image,
            '/vision/debug_image',
            10
        )

        # ---------------- MODEL ----------------
        torch.set_num_threads(self.TORCH_THREADS)

        self.get_logger().info("Loading YOLO model...")
        self.model = YOLO(self.MODEL_PATH)
        self.get_logger().info("Model ready")

        # ---------------- CAMERA SUBSCRIPTION ----------------
        self.create_subscription(
            Image,
            '/camera1/image_raw',
            self.image_callback,
            10
        )

    # ==================================================

    def send_cmd(self, linear, angular):

        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular

        self.cmd_pub.publish(msg)

    # ==================================================

    def image_callback(self, msg):

        # frame skipping for CPU reduction
        self.frame_count += 1
        if self.frame_count % self.FRAME_SKIP != 0:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # -------- YOLO inference --------
        with torch.no_grad():
            results = self.model(
                frame,
                imgsz=self.IMG_SIZE,
                conf=self.CONF_THRESH,
                verbose=False
            )

        boxes = results[0].boxes
        best_box = self.select_best_box(boxes)

        # -------- NO DETECTION --------
        if best_box is None:

            self.detect_pub.publish(Bool(data=False))

            # rotate slowly to search
            self.send_cmd(0.0, self.SEARCH_SPEED)

            return

        # -------- DETECTION FOUND --------
        x1, y1, x2, y2 = best_box

        center_x = float((x1 + x2) / 2.0)

        self.center_pub.publish(Float32(data=center_x))
        self.detect_pub.publish(Bool(data=True))

        error = center_x - self.TARGET_X

        # -------- MOVEMENT CONTROL --------
        if abs(error) < self.ALIGN_TOL:

            # aligned → drive forward
            self.send_cmd(self.FORWARD_SPEED, 0.0)

        elif error > 0:

            # target on right
            self.send_cmd(0.0, -self.TURN_SPEED)

        else:

            # target on left
            self.send_cmd(0.0, self.TURN_SPEED)

        # -------- DEBUG IMAGE --------
        annotated = results[0].plot()

        img_msg = self.bridge.cv2_to_imgmsg(
            annotated,
            encoding="bgr8"
        )

        self.image_pub.publish(img_msg)

    # ==================================================

    def select_best_box(self, boxes):

        if boxes is None or len(boxes) == 0:
            return None

        best_box = None
        best_area = -1

        for i, box in enumerate(boxes.xyxy):

            cls = int(boxes.cls[i].item())

            if cls != self.TARGET_CLASS:
                continue

            x1, y1, x2, y2 = box

            area = float((x2 - x1) * (y2 - y1))

            if area > best_area:
                best_area = area
                best_box = box

        return best_box


# ======================================================

def main():

    rclpy.init()

    node = EntryNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()