import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class DualCameraRelay(Node):

    def __init__(self):
        super().__init__('dual_camera_relay')

        # Camera1 relay
        self.sub1 = self.create_subscription(
            Image,
            '/camera1/image_raw',
            self.cam1_callback,
            10)

        self.pub1 = self.create_publisher(
            Image,
            '/camera1/image_relay',
            10)

        # Camera2 relay
        self.sub2 = self.create_subscription(
            Image,
            '/camera2/image_raw',
            self.cam2_callback,
            10)

        self.pub2 = self.create_publisher(
            Image,
            '/camera2/image_relay',
            10)

        self.get_logger().info("Dual camera relay started")

    def cam1_callback(self, msg):
        self.pub1.publish(msg)

    def cam2_callback(self, msg):
        self.pub2.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = DualCameraRelay()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()