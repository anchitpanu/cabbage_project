import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool
import time


class MovementNode(Node):

    def __init__(self):
        super().__init__('movement_node')

        # ---------------- Parameters ----------------
        self.linear_speed = 0.20
        self.distance_tolerance = 0.005

        self.PLANTER_LENGTH = 1.50
        self.ENTRY_MARGIN = 0.30
        self.STOP_DISTANCE = self.PLANTER_LENGTH + self.ENTRY_MARGIN
        self.OBSTACLE_THRESHOLD = 0.15

        # ---------------- Internal State ----------------
        self.current_distance = 0.0
        self.target_distance = 0.0
        self.moving = False

        self.vision_end = False
        self.obstacle_distance = 999.0

        # ---------------- Publishers ----------------
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

        # ---------------- Subscribers ----------------
        self.create_subscription(
            Float32,
            '/quin/distance_inside_planter',
            self.distance_callback,
            10
        )

        self.create_subscription(
            Bool,
            '/vision_end_detected',
            self.vision_callback,
            10
        )

        self.create_subscription(
            Float32,
            '/obstacle_distance',
            self.obstacle_callback,
            10
        )

        # ---------------- Control Timer ----------------
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("Movement Node Ready")

    # ==================================================
    # Callbacks
    # ==================================================

    def distance_callback(self, msg):
        self.current_distance = msg.data

    def vision_callback(self, msg):
        self.vision_end = msg.data

    def obstacle_callback(self, msg):
        self.obstacle_distance = msg.data

    # ==================================================
    # Public API
    # ==================================================

    def move_distance(self, meters):
        """
        Blocking movement call.
        Used by mission_node.
        """

        self.get_logger().info(f"Move {meters:.3f} m")

        self.reset_distance()
        time.sleep(0.1)

        self.target_distance = meters
        self.moving = True

        while rclpy.ok() and self.moving:
            rclpy.spin_once(self, timeout_sec=0.01)

        self.get_logger().info("Movement finished")

    def stop(self):
        self.publish_velocity(0.0)

    # ==================================================
    # Control Loop
    # ==================================================

    def control_loop(self):

        if not self.moving:
            return

        # ---------------- 🔴 Priority 1: Obstacle ----------------
        if self.obstacle_distance < self.OBSTACLE_THRESHOLD:
            self.get_logger().warn("EMERGENCY STOP: Obstacle")
            self.abort_motion()
            return

        # ---------------- 🟢 Priority 2: Vision End ----------------
        if self.vision_end:
            self.get_logger().info("STOP: Vision End")
            self.abort_motion()
            return

        # ---------------- 🟡 Priority 3: Encoder Safety ----------------
        if self.current_distance >= self.STOP_DISTANCE:
            self.get_logger().warn("STOP: Encoder Safety Limit")
            self.abort_motion()
            return

        # ---------------- Target Distance ----------------
        error = self.target_distance - self.current_distance

        if error <= self.distance_tolerance:
            self.abort_motion()
            return

        # Normal forward movement
        self.publish_velocity(self.linear_speed)

    # ==================================================
    # Helpers
    # ==================================================

    def publish_velocity(self, linear_x):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)

    def abort_motion(self):
        self.publish_velocity(0.0)
        self.moving = False

    def reset_distance(self):
        msg = Twist()
        self.reset_pub.publish(msg)


# ======================================================
# Main
# ======================================================

def main(args=None):
    rclpy.init(args=args)
    node = MovementNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()