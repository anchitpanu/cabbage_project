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

        # Slowdown zone: reduce speed when within this distance of target
        self.SLOWDOWN_ZONE = 0.10
        self.MIN_SPEED = 0.05

        # Timeout: abort if move takes longer than this (seconds)
        self.MOVE_TIMEOUT = 180.0

        # ---------------- Internal State ----------------
        self._current_distance = 0.0
        self.target_delta = 0.0
        self.start_distance = 0.0
        self.moving = False
        self._move_start_time = None

        # FIX: reset when first distance message arrives from Arduino
        self._arduino_ready = False      # becomes True after first distance msg
        self._reset_confirmed = False    # becomes True after distance reads ~0

        # ---------------- Publishers ----------------
        self.cmd_pub = self.create_publisher(
            Twist,
            '/quin/move_command',
            10
        )

        self.done_pub = self.create_publisher(
            Bool,
            '/quin/move_done',
            10
        )

        self.reset_pub = self.create_publisher(
            Bool,
            '/quin/reset_distance',
            10
        )

        # ---------------- Subscribers ----------------
        # BEST_EFFORT QoS to match Arduino publisher
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
            Float32,
            '/quin/cmd_move',
            self.move_command_callback,
            10
        )

        # ---------------- Timers ----------------
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("Movement Node Ready — waiting for Arduino...")

    # ==================================================
    # Distance Callback — reset happens here on first message
    # ==================================================

    def distance_callback(self, msg):
        self._current_distance = msg.data

        # FIX: first time we receive distance from Arduino → send reset
        if not self._arduino_ready:
            self._arduino_ready = True
            self.get_logger().info(
                f"Arduino connected — distance was {msg.data:.3f} m, sending reset..."
            )
            self._publish_reset()
            return

        # After reset sent, check if it confirmed
        if not self._reset_confirmed:
            if abs(self._current_distance) < 0.01:
                self._reset_confirmed = True
                self.get_logger().info("Encoder reset confirmed — ready to move")
            else:
                # Still not 0, keep sending reset
                self._publish_reset()

    # ==================================================
    # Thread-safe distance property
    # ==================================================

    @property
    def current_distance(self):
        return self._current_distance

    # ==================================================
    # Callbacks
    # ==================================================

    def move_command_callback(self, msg):
        """Receives a Float32 move request (meters) from MissionNode."""

        # Block movement until encoder is reset and confirmed
        if not self._reset_confirmed:
            self.get_logger().warn("Move command received but encoder not ready yet — ignored")
            return

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

        # ---------------- Total Distance Safety ----------------
        if self.current_distance >= self.MAX_TRAVEL_DISTANCE:
            self.get_logger().warn("STOP: Max planter distance reached")
            self.abort_motion(success=False)
            return

        # ---------------- Timeout Check ----------------
        elapsed = (self.get_clock().now() - self._move_start_time).nanoseconds / 1e9
        if elapsed > self.MOVE_TIMEOUT:
            self.get_logger().error(
                f"ABORT: Move timed out after {elapsed:.1f}s"
            )
            self.abort_motion(success=False)
            return

        # ---------------- Target Check ----------------
        travelled = self.current_distance - self.start_distance
        error = self.target_delta - travelled

        if error <= self.distance_tolerance:
            self.get_logger().info(
                f"Target reached — travelled: {travelled:.3f} m"
            )
            self.abort_motion(success=True)
            return

        # ---------------- Ramped Speed (approach slowdown) ----------------
        speed = self._compute_speed(error)
        self.publish_velocity(speed)

    # ==================================================
    # Helpers
    # ==================================================

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