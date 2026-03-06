import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge
from pupil_apriltags import Detector
import cv2
import threading
import numpy as np


class AprilTagNode(Node):
    def __init__(self):
        super().__init__('apriltag_node')

        self.bridge = CvBridge()
        self.frame = None
        self.frame_lock = threading.Lock()  # Thread-safe frame access
        self.last_tag = None

        # Use a callback group to avoid blocking
        self.cb_group = ReentrantCallbackGroup()

        # Subscriber
        self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10,
            callback_group=self.cb_group
        )

        # Publisher
        self.param_pub = self.create_publisher(
            Int32MultiArray,
            '/mission_params',
            10
        )

        # AprilTag detector — use tagStandard52h13 or change to tag36h11 if needed
        self.detector = Detector(
            families="tagStandard52h13",
            nthreads=2,
            quad_decimate=2.0,   # Reduced from 3.0 for better detection
            quad_sigma=0.8,      # Slight blur helps detection
            refine_edges=1,
            decode_sharpening=0.25
        )

        # Timer: detect at 5 Hz
        self.timer = self.create_timer(
            0.2,
            self.detect_tag,
            callback_group=self.cb_group
        )

        self.get_logger().info("AprilTag Node Ready")

    def image_callback(self, msg):
        """Store the latest frame in a thread-safe way."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.frame_lock:
                # Store a copy to avoid memory issues with ROS message buffers
                self.frame = frame.copy()
        except Exception as e:
            self.get_logger().warn(f"Image conversion failed: {e}")

    def detect_tag(self):
        """Detect AprilTags at a limited rate."""
        with self.frame_lock:
            if self.frame is None:
                return
            frame = self.frame.copy()  # Work on a local copy

        try:
            # Resize to reduce CPU load
            frame_resized = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_LINEAR)
            gray = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2GRAY)

            # Normalize contrast — helps in poor lighting
            #gray = cv2.equalizeHist(gray)

            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
            gray = clahe.apply(gray)

            results = self.detector.detect(gray)

            if not results:
                return

            # Use the detection with the highest margin (most confident)
            best = max(results, key=lambda r: r.decision_margin)

            if best.decision_margin < 10.0:
                self.get_logger().debug("Tag detected but margin too low, skipping")
                return

            tag_id = best.tag_id

            # Only publish on new tag
            if tag_id == self.last_tag:
                return

            self.last_tag = tag_id
            self.get_logger().info(f"New tag detected: ID={tag_id}")

            decoded = self.decode_tag(tag_id)
            if decoded is None:
                self.get_logger().warn(f"Tag ID {tag_id} could not be decoded (invalid C unit)")
                return

            AB, C, DE = decoded

            msg_out = Int32MultiArray()
            msg_out.data = [AB, C, DE]
            self.param_pub.publish(msg_out)

            self.get_logger().info(
                f"Mission published: AB={AB}cm, C={C}cm, DE={DE}cm"
            )

        except Exception as e:
            self.get_logger().error(f"Detection error: {e}")

    def decode_tag(self, tag_id):
        """
        Decode tag ID into mission parameters.
        Format: ABCDE (5-digit number)
          AB  = tag_id // 1000       → distance in cm
          C   = (tag_id // 100) % 10 → spacing code (mapped)
          DE  = tag_id % 100         → another distance in cm
        """
        AB = tag_id // 1000
        C_unit = (tag_id // 100) % 10
        DE = tag_id % 100

        spacing_map = {1: 5, 2: 10, 3: 15, 4: 20, 5: 25}
        if C_unit not in spacing_map:
            return None

        C = spacing_map[C_unit]
        return AB, C, DE


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagNode()

    # MultiThreadedExecutor prevents timer and subscriber from blocking each other
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down AprilTag Node")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()