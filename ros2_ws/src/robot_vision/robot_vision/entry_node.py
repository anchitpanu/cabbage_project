import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool
from cv_bridge import CvBridge
import torch
from ultralytics import YOLO


class EntryNode(Node):

    def __init__(self):
        super().__init__('entry_node')

        # ---------------- Parameters ----------------
        self.MODEL_PATH = '/home/jorjeen/plant/cabbage_project/entering_model/enterbest.pt'
        self.CONF_THRESH = 0.4
        self.IMG_SIZE = 416
        self.TARGET_CLASS = 0
        self.TORCH_THREADS = 4

        # ---------------- Tools ----------------
        self.bridge = CvBridge()

        # ---------------- Publishers ----------------
        self.center_pub = self.create_publisher(
            Float32,
            '/tag_center_x',
            10
        )

        self.detection_pub = self.create_publisher(
            Bool,
            '/vision/detection_active',
            10
        )

        self.image_pub = self.create_publisher(
            Image,
            '/vision/debug_image',
            10
        )

        # ---------------- Model ----------------
        torch.set_num_threads(self.TORCH_THREADS)

        self.get_logger().info(
            f"Loading YOLO model from {self.MODEL_PATH}"
        )

        self.model = YOLO(self.MODEL_PATH)

        self.get_logger().info("Model loaded")

        # ---------------- Image Subscription ----------------
        self.subscription = self.create_subscription(
            Image,
            '/quin/image_raw',
            self.image_callback,
            10
        )

    # ==================================================

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )

        # ---------------- YOLO Inference ----------------
        results = self.model(
            frame,
            imgsz=self.IMG_SIZE,
            conf=self.CONF_THRESH,
            verbose=False
        )

        boxes = results[0].boxes
        best_box = self._select_best_box(boxes)

        # ---------------- Detection logic ----------------
        if best_box is not None:

            x1, y1, x2, y2 = best_box
            center_x = float((x1 + x2) / 2.0)

            msg = Float32()
            msg.data = center_x

            self.center_pub.publish(msg)

            self._publish_detection_active(True)

            self.get_logger().info(
                f"Apriltag detected | center_x={center_x:.1f}"
            )

            annotated = results[0].plot()

            img_msg = self.bridge.cv2_to_imgmsg(
                annotated,
                encoding="bgr8"
            )

            self.image_pub.publish(img_msg)

        else:

            self._publish_detection_active(False)

    # ==================================================

    def _select_best_box(self, boxes):

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

    # ==================================================

    def _publish_detection_active(self, active):

        msg = Bool()
        msg.data = active

        self.detection_pub.publish(msg)


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