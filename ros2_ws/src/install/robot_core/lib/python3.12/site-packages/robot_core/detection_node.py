import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String


class DetectionNode(Node):

    def __init__(self):
        super().__init__('detection_node')

        self.harvest_size = 10.0

        self.sub = self.create_subscription(
            Float32,
            '/cabbage_detection',
            self.callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            String,
            '/cabbage_action',
            10
        )

        self.get_logger().info("Detection Node Ready")

    # ---------------------------------

    def callback(self, msg):

        size = msg.data

        cmd = String()

        if size == 0.0:
            cmd.data = "MOVE_FORWARD"

        elif size >= self.harvest_size:
            cmd.data = "HARVEST"

        else:
            cmd.data = "WAIT_GROW"

        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f"Cabbage size: {size:.1f} → {cmd.data}"
        )


def main(args=None):

    rclpy.init(args=args)

    node = DetectionNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()