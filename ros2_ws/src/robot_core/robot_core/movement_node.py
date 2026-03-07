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

        # Slowdown zone: reduce speed when within this distance of target
        self.SLOWDOWN_ZONE = 0.10
        self.MIN_SPEED = 0.05

        # Timeout: abort if move takes longer than this (seconds)
        self.MOVE_TIMEOUT = 30.0

        # Obstacle sensor watchdog: if no obstacle msg received within this
        # time, treat it as unsafe (sensor dead)
        self.OBSTACLE_WATCHDOG_TIMEOUT = 1.0

        # ---------------- Internal State ----------------
        self._current_distance = 0.0   # written only in distance_callback
        self.target_delta = 0.0
        self.start_distance = 0.0
        self.moving = False

        # FIX: default to 0.0 (unsafe) instead of 999.0
        # Robot will not move until the obstacle sensor publishes at least once
        self.obstacle_distance = 0.0
        self.obstacle_last_update = None   # tracks last sensor msg time

        self._move_start_time = None       # for timeout tracking

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
    # Thread-safe distance property
    # ==================================================

    @property
    def current_distance(self):
        return self._current_distance

    # ==================================================
    # Callbacks
    # ==================================================

    def distance_callback(self, msg):
        # Atomic float assignment — safe in Python's GIL
        self._current_distance = msg.data

    def obstacle_callback(self, msg):
        self.obstacle_distance = msg.data
        self.obstacle_last_update = self.get_clock().now()

    def move_command_callback(self, msg):
        """Receives a move request (meters) from MissionNode over ROS topic."""
        meters = msg.data

        if self.moving:
            self.get_logger().warn("Received move command while already moving — ignored")
            return

        # FIX: Check obstacle sensor is alive before allowing movement
        if not self._obstacle_sensor_alive():
            self.get_logger().error(
                "Move rejected: obstacle sensor not publishing. Check /obstacle_distance"
            )
            self.publish_done(success=False)
            return

        # FIX: Check obstacle sensor is clear before starting
        if self.obstacle_distance < self.OBSTACLE_THRESHOLD:
            self.get_logger().error(
                "Move rejected: obstacle detected before starting"
            )
            self.publish_done(success=False)
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

        self.get_logger().info(f"Moving {meters:.3f} m")

    # ==================================================
    # Control Loop
    # ==================================================

    def control_loop(self):

        if not self.moving:
            return

        # ---------------- Obstacle Sensor Watchdog ----------------
        # FIX: If obstacle sensor goes silent mid-move, emergency stop
        if not self._obstacle_sensor_alive():
            self.get_logger().error("EMERGENCY STOP: Obstacle sensor timeout")
            self.abort_motion(success=False)
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

        # ---------------- Timeout Check ----------------
        # FIX: Abort if move is taking too long (sensor failure, wheel slip, etc.)
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
            self.get_logger().info("Target distance reached")
            self.abort_motion(success=True)
            return

        # ---------------- Ramped Speed (approach slowdown) ----------------
        # FIX: Slow down when close to target to avoid overshoot
        speed = self._compute_speed(error)
        self.publish_velocity(speed)

    # ==================================================
    # Helpers
    # ==================================================

    def _obstacle_sensor_alive(self) -> bool:
        """Returns True only if obstacle sensor has published recently."""
        if self.obstacle_last_update is None:
            return False
        age = (self.get_clock().now() - self.obstacle_last_update).nanoseconds / 1e9
        return age < self.OBSTACLE_WATCHDOG_TIMEOUT

    def _compute_speed(self, error: float) -> float:
        """
        Ramp speed down as the robot approaches the target.
        Full speed outside SLOWDOWN_ZONE, linearly scaled down inside it.
        """
        if error >= self.SLOWDOWN_ZONE:
            return self.linear_speed

        # Linear interpolation between MIN_SPEED and linear_speed
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