import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')

        self.publisher = self.create_publisher(
            Image,
            '/quin/image_raw',
            10
        )

        self.bridge = CvBridge()

        # Open USB camera
        self.cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)

        if not self.cap.isOpened():
            self.get_logger().error("Cannot open camera")
            return

        # Set resolution (good for AprilTag speed)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        self.timer = self.create_timer(0.03, self.timer_callback)  # ~30 FPS

        self.get_logger().info("Camera node started")

    def timer_callback(self):

        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warn("Failed to read frame")
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")

        self.publisher.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = CameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()