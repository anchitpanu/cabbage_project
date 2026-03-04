import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool


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
        self.target_delta = 0.0
        self.start_distance = 0.0
        self.moving = False

        self.obstacle_distance = 999.0

        # ---------------- Publishers ----------------
        self.cmd_pub = self.create_publisher(
            Twist,
            '/quin/cmd_move',
            10
        )

        # Signals MissionNode when a move is complete (True) or aborted (False)
        self.done_pub = self.create_publisher(
            Bool,
            '/quin/move_done',
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

        # MissionNode sends a distance (meters) here to trigger movement
        self.create_subscription(
            Float32,
            '/quin/move_command',
            self.move_command_callback,
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

    def move_command_callback(self, msg):
        """Receives a move request (meters) from MissionNode over ROS topic."""
        meters = msg.data

        if self.moving:
            self.get_logger().warn("Received move command while already moving — ignored")
            return

        # Safety clamp
        if self.current_distance + meters > self.MAX_TRAVEL_DISTANCE:
            meters = self.MAX_TRAVEL_DISTANCE - self.current_distance
            self.get_logger().warn(
                f"Clamped distance to {meters:.3f} m due to safety limit"
            )

        if meters <= 0.0:
            self.get_logger().warn("Move command <= 0, nothing to do")
            self.publish_done(success=True)
            return

        self.start_distance = self.current_distance
        self.target_delta = meters
        self.moving = True

        self.get_logger().info(f"Moving {meters:.3f} m")

    # ==================================================
    # Control Loop
    # ==================================================

    def control_loop(self):

        if not self.moving:
            return

        # ---------------- Obstacle Safety ----------------
        if self.obstacle_distance < self.OBSTACLE_THRESHOLD:
            self.get_logger().warn("EMERGENCY STOP: Obstacle detected")
            self.abort_motion(success=False)
            return

        # ---------------- Total Distance Safety ----------------
        if self.current_distance >= self.MAX_TRAVEL_DISTANCE:
            self.get_logger().warn("STOP: Max planter distance reached")
            self.abort_motion(success=False)
            return

        # ---------------- Target Check ----------------
        travelled = self.current_distance - self.start_distance
        error = self.target_delta - travelled

        if error <= self.distance_tolerance:
            self.get_logger().info("Target distance reached")
            self.abort_motion(success=True)
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

    def abort_motion(self, success: bool):
        self.publish_velocity(0.0)
        self.moving = False
        self.publish_done(success=success)

    def publish_done(self, success: bool):
        msg = Bool()
        msg.data = success
        self.done_pub.publish(msg)
        self.get_logger().info(f"Move done — success={success}")


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