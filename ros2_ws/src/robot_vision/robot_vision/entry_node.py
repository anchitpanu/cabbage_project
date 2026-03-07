import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool
from cv_bridge import CvBridge
import cv2
import torch
from ultralytics import YOLO


class VisionNode(Node):

    def __init__(self):
        super().__init__('entry_node')

        # ---------------- Parameters ----------------
        self.MODEL_PATH    = '/home/quin/cabbage_project/cab_model/best.pt'
        self.CONF_THRESH   = 0.4
        self.IMG_SIZE      = 416
        self.TARGET_CLASS  = 0        # class index for cabbage — adjust if needed
        self.TORCH_THREADS = 4
        self.TIMER_HZ      = 10.0     # FIX: reduced from 20 Hz to avoid CPU overload

        # ---------------- Internal State ----------------
        self.bridge = CvBridge()
        self._last_inference_ok = True

        # ---------------- Publishers ----------------
        # Publishes center X of best detection (pixels)
        self.center_pub = self.create_publisher(
            Float32,
            '/tag_center_x',
            10
        )

        # FIX: Publishes whether a detection is currently present
        # Subscribers can use this to know when detection is lost
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
        self.get_logger().info(f"Loading YOLO model from {self.MODEL_PATH} ...")
        self.model = YOLO(self.MODEL_PATH)
        self.get_logger().info("Model loaded")

        # ---------------- Camera ----------------
        self.cap = cv2.VideoCapture(0)

        # FIX: Check camera opened successfully
        if not self.cap.isOpened():
            self.get_logger().error(
                "Camera failed to open on index 0. Check connection and permissions."
            )
            raise RuntimeError("Camera not available")

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.get_logger().info("Camera opened successfully")

        # ---------------- Timer ----------------
        self.timer = self.create_timer(1.0 / self.TIMER_HZ, self.loop)

        self.get_logger().info("Vision Node Ready")

    # ==================================================
    # Main Loop
    # ==================================================

    def loop(self):
        ret, frame = self.cap.read()

        if not ret:
            if self._last_inference_ok:
                self.get_logger().warn("Camera read failed — check connection")
                self._last_inference_ok = False
            self._publish_detection_active(False)
            return

        self._last_inference_ok = True

        # ---------------- YOLO Inference ----------------
        results = self.model(frame, imgsz=self.IMG_SIZE, conf=self.CONF_THRESH)
        boxes = results[0].boxes

        best_box = self._select_best_box(boxes)

        # ---------------- Publish Center X ----------------
        if best_box is not None:
            x1, y1, x2, y2 = best_box
            center_x = float((x1 + x2) / 2.0)

            msg = Float32()
            msg.data = center_x
            self.center_pub.publish(msg)
            self._publish_detection_active(True)
        else:
            # FIX: Explicitly publish that no detection is active
            # so subscribers are never left hanging
            self._publish_detection_active(False)

        # ---------------- Debug Image ----------------
        annotated = results[0].plot()
        img_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        self.image_pub.publish(img_msg)

    # ==================================================
    # Box Selection
    # ==================================================

    def _select_best_box(self, boxes):
        """
        FIX: Selects the LARGEST bounding box (closest / most prominent detection)
        instead of the smallest.
        Also filters by target class index to avoid false positives.
        Returns the (x1, y1, x2, y2) tensor of the best box, or None.
        """
        if boxes is None or len(boxes) == 0:
            return None

        best_box  = None
        best_area = -1

        for i, box in enumerate(boxes.xyxy):
            # FIX: Filter by class
            cls = int(boxes.cls[i].item())
            if cls != self.TARGET_CLASS:
                continue

            x1, y1, x2, y2 = box
            area = float((x2 - x1) * (y2 - y1))

            if area > best_area:
                best_area = area
                best_box  = box

        return best_box

    # ==================================================
    # Helpers
    # ==================================================

    def _publish_detection_active(self, active: bool):
        msg = Bool()
        msg.data = active
        self.detection_pub.publish(msg)

    # ==================================================
    # Shutdown
    # ==================================================

    def destroy_node(self):
        # FIX: Release camera on shutdown so it's not left locked
        if self.cap.isOpened():
            self.cap.release()
            self.get_logger().info("Camera released")
        super().destroy_node()


# ======================================================
# Main
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