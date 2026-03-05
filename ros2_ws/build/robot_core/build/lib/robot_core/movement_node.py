import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


class MovementNode(Node):

    def __init__(self):
        super().__init__('movement_node')

        # ---------------- Parameters ----------------
        self.linear_speed = 0.20
        self.distance_tolerance = 0.005
        self.MAX_TRAVEL_DISTANCE = 1.70
        self.OBSTACLE_THRESHOLD = 0.15

        # ---------------- Internal State ----------------
        self.current_distance = 0.0
        self.target_distance = 0.0
        self.start_distance = 0.0
        self.moving = False

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

    def obstacle_callback(self, msg):
        self.obstacle_distance = msg.data

    # ==================================================
    # Public API (for mission node)
    # ==================================================

    def move_distance(self, meters):

        if self.moving:
            self.get_logger().warn("Already moving")
            return

        # safety check
        if self.current_distance + meters > self.MAX_TRAVEL_DISTANCE:
            meters = self.MAX_TRAVEL_DISTANCE - self.current_distance
            self.get_logger().warn(
                f"Clamped distance to {meters:.3f} m due to safety limit"
            )

        self.start_distance = self.current_distance
        self.target_distance = meters
        self.moving = True

        self.get_logger().info(f"Start move {meters:.3f} m")

    def stop(self):
        self.abort_motion()

    # ==================================================
    # Control Loop
    # ==================================================

    def control_loop(self):

        if not self.moving:
            return

        # ---------------- Obstacle Safety ----------------
        if self.obstacle_distance < self.OBSTACLE_THRESHOLD:
            self.get_logger().warn("EMERGENCY STOP: Obstacle")
            self.abort_motion()
            return

        # ---------------- Total Distance Safety ----------------
        if self.current_distance >= self.MAX_TRAVEL_DISTANCE:
            self.get_logger().warn("STOP: Max planter distance reached")
            self.abort_motion()
            return

        # ---------------- Target Distance ----------------
        travelled = self.current_distance - self.start_distance
        error = self.target_distance - travelled

        if error <= self.distance_tolerance:
            self.get_logger().info("Target distance reached")
            self.abort_motion()
            return

        # ---------------- Move Forward ----------------
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