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
        self.MOVE_TIMEOUT = 30.0

        # ---------------- Internal State ----------------
        self._current_distance = 0.0
        self.target_delta = 0.0
        self.start_distance = 0.0
        self.moving = False
        self._move_start_time = None

        # Reset state
        self._arduino_ready = False     # True after first distance msg received
        self._reset_confirmed = False   # True after distance confirms ~0

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

        # Publishes True when encoder is reset and robot is ready to move
        self.ready_pub = self.create_publisher(
            Bool,
            '/quin/robot_ready',
            10
        )

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
            Float32,
            '/quin/cmd_move',
            self.move_command_callback,
            10
        )

        # Receives manual reset trigger from MissionNode
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

        # First message received — Arduino is now connected
        if not self._arduino_ready:
            self._arduino_ready = True
            self.get_logger().info(
                f"Arduino connected — distance is {msg.data:.3f} m (waiting for mission reset trigger)"
            )
            return  # Do NOT reset here — wait for mission_node to trigger it

        # If a reset was triggered, keep sending reset until encoder confirms ~0
        if not self._reset_confirmed:
            if abs(self._current_distance) < 0.01:
                self._reset_confirmed = True
                self.get_logger().info("Encoder reset confirmed — robot is ready!")

                # Notify MissionNode that robot is ready to move
                ready_msg = Bool()
                ready_msg.data = True
                self.ready_pub.publish(ready_msg)
            else:
                # Still not 0, keep sending reset
                self._publish_reset()

    # ==================================================
    # Manual Reset Trigger (called by MissionNode after params received)
    # ==================================================

    def trigger_reset_callback(self, msg: Bool):
        if msg.data:
            if self.moving:
                self.get_logger().warn("Reset trigger ignored — robot is currently moving")
                return
            self._reset_confirmed = False
            self._publish_reset()
            self.get_logger().info("Reset triggered by mission — waiting for encoder to confirm 0...")

    # ==================================================
    # Thread-safe distance property
    # ==================================================

    @property
    def current_distance(self):
        return self._current_distance

    # ==================================================
    # Move Command Callback
    # ==================================================

    def move_command_callback(self, msg):
        """Receives a Float32 move request (meters) from MissionNode."""

        if not self._reset_confirmed:
            self.get_logger().warn("Move command received but encoder not ready yet — ignored")
            return

        if self.moving:
            self.get_logger().warn("Received move command while already moving — ignored")
            return

        meters = msg.data

        # Safety clamp — uses absolute position to enforce total distance limit
        if self.current_distance + meters > self.MAX_TRAVEL_DISTANCE:
            meters = self.MAX_TRAVEL_DISTANCE - self.current_distance
            self.get_logger().warn(
                f"Clamped distance to {meters:.3f} m due to safety limit"
            )

        if meters <= 0.0:
            self.get_logger().warn("Move command <= 0, nothing to do")
            self.publish_done(success=True)
            return

        # Snapshot absolute position — travelled is calculated relative to this
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
        # travelled = relative distance for THIS move only
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