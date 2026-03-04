import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge

import cv2
import apriltag


class AprilTagNode(Node):

    def __init__(self):
        super().__init__('apriltag_node')

        self.bridge = CvBridge()

        self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.param_pub = self.create_publisher(
            Int32MultiArray,
            '/mission_params',
            10
        )

        options = apriltag.DetectorOptions(families='tag52h3')
        self.detector = apriltag.Detector(options)

        self.last_tag = None

        self.get_logger().info("AprilTag Node Ready")

    def decode_tag(self, tag_id):

        AB = tag_id // 1000
        C_Unit  = (tag_id // 100) % 10
        DE = tag_id % 100

        spacing_map = {1:5, 2:10, 3:15, 4:20, 5:25}

        if C_Unit not in spacing_map:
            return None
        
        C = spacing_map[C_Unit]

        return AB, C, DE

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        results = self.detector.detect(gray)
        if not results:
            return

        tag_id = results[0].tag_id

        if tag_id == self.last_tag:
            return

        self.last_tag = tag_id

        decoded = self.decode_tag(tag_id)
        if decoded is None:
            return

        AB, C, DE = decoded

        msg_out = Int32MultiArray()
        msg_out.data = [AB, C, DE]

        self.param_pub.publish(msg_out)

        self.get_logger().info(
            f"Mission: AB={AB}cm, C={C}cm, DE={DE}cm"
        )


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()