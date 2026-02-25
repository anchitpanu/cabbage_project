import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist
import time


class EntryController(Node):

    def __init__(self):
        super().__init__('entry_controller')

        self.edge_detected = False
        self.angle_error = 0.0

        self.cmd_pub = self.create_publisher(
            Twist,
            '/quin/cmd_move',
            10
        )

        self.reset_pub = self.create_publisher(
            Twist,
            '/quin/reset_distance',
            10
        )

        self.create_subscription(
            Bool,
            '/planter_edge_detected',
            self.edge_callback,
            10
        )

        self.create_subscription(
            Float32,
            '/planter_angle_error',
            self.angle_callback,
            10
        )

    # ------------------------------------------
    # Callbacks
    # ------------------------------------------

    def edge_callback(self, msg):
        self.edge_detected = msg.data

    def angle_callback(self, msg):
        self.angle_error = msg.data

    # ------------------------------------------
    # Public Entry Function
    # ------------------------------------------

    def execute_entry(self):

        self.get_logger().info("Starting entry procedure")

        # ---- ALIGN ----
        while rclpy.ok():

            rclpy.spin_once(self, timeout_sec=0.01)

            if not self.edge_detected:
                self.publish_cmd(0.0, 0.2)
                continue

            if abs(self.angle_error) > 3.0:
                direction = -0.2 if self.angle_error > 0 else 0.2
                self.publish_cmd(0.0, direction)
            else:
                self.stop_robot()
                break

        self.get_logger().info("Aligned with planter")

        time.sleep(0.5)

        # ---- ENTER ----
        self.get_logger().info("Entering planter")

        self.publish_cmd(0.15, 0.0)
        time.sleep(2.0)   # Adjust later with encoder
        self.stop_robot()

        # Reset encoder after entering
        self.reset_pub.publish(Twist())
        time.sleep(0.2)

        self.get_logger().info("Entry complete")

    # ------------------------------------------

    def publish_cmd(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_pub.publish(msg)

    def stop_robot(self):
        self.publish_cmd(0.0, 0.0)