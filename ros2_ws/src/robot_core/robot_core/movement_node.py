import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool
from rclpy.qos import QoSProfile, ReliabilityPolicy


class MovementNode(Node):

    def __init__(self):
        super().__init__('movement_node')

        # ---------------- Parameters ----------------
        self.linear_speed = 0.20
        self.distance_tolerance = 0.005
        self.MAX_TRAVEL_DISTANCE = 1.70

        self.SLOWDOWN_ZONE = 0.10
        self.MIN_SPEED = 0.05

        self.MOVE_TIMEOUT = 30.0

        # ---------------- Internal State ----------------
        self._current_distance = 0.0
        self.target_delta = 0.0
        self.start_distance = 0.0
        self.moving = False
        self._move_start_time = None

        self._arduino_ready = False
        self._reset_confirmed = False

        # ---------------- Publishers ----------------
        self.cmd_pub = self.create_publisher(
            Twist, '/quin/move_command', 10)

        self.done_pub = self.create_publisher(
            Bool, '/quin/move_done', 10)

        self.reset_pub = self.create_publisher(
            Bool, '/quin/trigger_reset', 10)

        # ---------------- Subscribers ----------------
        encoder_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.create_subscription(
            Float32,
            '/quin/distance_inside_planter',
            self.distance_callback,
            encoder_qos
        )

        self.create_subscription(
            Bool,
            '/quin/robot_ready',
            self.robot_ready_from_esp_callback,
            encoder_qos
        )

        self.create_subscription(
            Float32,
            '/quin/cmd_move',
            self.move_command_callback,
            10
        )

        self.create_subscription(
            Bool,
            '/quin/trigger_reset',
            self.trigger_reset_callback,
            10
        )

        # ---------------- Timers ----------------
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("Movement Node Ready — waiting for Arduino...")

    # ==================================================
    # Distance Callback
    # ==================================================

    def distance_callback(self, msg):
        self._current_distance = msg.data

        if not self._arduino_ready:
            self._arduino_ready = True
            self.get_logger().info(
                f"Arduino connected — distance is {msg.data:.3f} m"
            )

        if not self._reset_confirmed:
            if abs(self._current_distance) < 0.01:
                self._reset_confirmed = True
                self.get_logger().info("Encoder reset confirmed locally")
            else:
                self._publish_reset()

    # ==================================================
    # Robot Ready from ESP32
    # ==================================================

    def robot_ready_from_esp_callback(self, msg: Bool):

        if not msg.data:
            return

        if self._reset_confirmed:
            return

        self._reset_confirmed = True
        self.get_logger().info("Robot ready confirmed from ESP32")

        
    # ==================================================
    # Manual Reset Trigger
    # ==================================================

    def trigger_reset_callback(self, msg: Bool):

        if not msg.data:
            return

        # ถ้ากำลังรอ reset อยู่แล้ว → ignore
        if not self._reset_confirmed:
            return

        if self.moving:
            self.get_logger().warn("Reset ignored — robot is moving")
            return

        self._reset_confirmed = False
        self._publish_reset()

        self.get_logger().info(
            "Reset triggered by mission — waiting for ESP32 confirmation..."
        )

    # ==================================================
    # Move Command Callback
    # ==================================================

    def move_command_callback(self, msg):
        if not self._reset_confirmed:
            self.get_logger().warn("Move command received but encoder not ready yet — ignored")
            return

        if self.moving:
            self.get_logger().warn("Received move command while already moving — ignored")
            return

        meters = msg.data

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
        self._move_start_time = self.get_clock().now()

        self.get_logger().info(
            f"Moving {meters:.3f} m  (start distance: {self.start_distance:.3f} m)"
        )

    # ==================================================
    # Control Loop
    # ==================================================

    def control_loop(self):
        if not self.moving:
            return

        if self.current_distance >= self.MAX_TRAVEL_DISTANCE:
            self.get_logger().warn("STOP: Max planter distance reached")
            self.abort_motion(success=False)
            return

        elapsed = (self.get_clock().now() - self._move_start_time).nanoseconds / 1e9
        if elapsed > self.MOVE_TIMEOUT:
            self.get_logger().error(
                f"ABORT: Move timed out after {elapsed:.1f}s"
            )
            self.abort_motion(success=False)
            return

        travelled = self.current_distance - self.start_distance
        error = self.target_delta - travelled

        if error <= self.distance_tolerance:
            self.get_logger().info(
                f"Target reached — travelled: {travelled:.3f} m"
            )
            self.abort_motion(success=True)
            return

        speed = self._compute_speed(error)
        self.publish_velocity(speed)

    # ==================================================
    # Helpers
    # ==================================================

    @property
    def current_distance(self):
        return self._current_distance

    def _compute_speed(self, error: float) -> float:
        if error >= self.SLOWDOWN_ZONE:
            return self.linear_speed
        ratio = error / self.SLOWDOWN_ZONE
        speed = self.MIN_SPEED + ratio * (self.linear_speed - self.MIN_SPEED)
        return max(speed, self.MIN_SPEED)

    def publish_velocity(self, linear_x):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)

    def abort_motion(self, success: bool):
        self.publish_velocity(0.0)
        self.moving = False
        self._move_start_time = None
        self.publish_done(success=success)

    def publish_done(self, success: bool):
        msg = Bool()
        msg.data = success
        self.done_pub.publish(msg)
        self.get_logger().info(f"Move done — success={success}")

    def _publish_reset(self):
        msg = Bool()
        msg.data = True
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