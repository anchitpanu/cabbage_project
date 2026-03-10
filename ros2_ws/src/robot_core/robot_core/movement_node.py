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

        self.MOVE_TIMEOUT = 180.0

        # obstacle safety
        self.OBSTACLE_THRESHOLD = 0.25
        self.OBSTACLE_TIMEOUT = 1.0

        # ---------------- Internal State ----------------
        self._current_distance = 0.0
        self.target_delta = 0.0
        self.start_distance = 0.0
        self.moving = False
        self._move_start_time = None

        # Arduino reset handling
        self._arduino_ready = False
        self._reset_confirmed = False

        # obstacle sensor
        self.obstacle_distance = 999.0
        self.obstacle_last_update = None

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
            '/obstacle_distance',
            self.obstacle_callback,
            10
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
    # Distance Callback
    # ==================================================

    def distance_callback(self, msg):
        self._current_distance = msg.data

        if not self._arduino_ready:
            self._arduino_ready = True
            self.get_logger().info(
                f"Arduino connected — distance {msg.data:.3f} m, sending reset"
            )
            self._publish_reset()
            return

        if not self._reset_confirmed:
            if abs(self._current_distance) < 0.01:
                self._reset_confirmed = True
                self.get_logger().info("Encoder reset confirmed")
            else:
                self._publish_reset()

    def obstacle_callback(self, msg):
        self.obstacle_distance = msg.data
        self.obstacle_last_update = self.get_clock().now()

    @property
    def current_distance(self):
        return self._current_distance

    # ==================================================
    # Move Command
    # ==================================================

    def move_command_callback(self, msg):

        if not self._reset_confirmed:
            self.get_logger().warn("Move ignored — encoder not ready")
            return

        meters = msg.data

        if self.moving:
            self.get_logger().warn("Already moving — ignored")
            return

        if not self._obstacle_sensor_alive():
            self.get_logger().error("Move rejected: obstacle sensor offline")
            self.publish_done(False)
            return

        if self.obstacle_distance < self.OBSTACLE_THRESHOLD:
            self.get_logger().error("Move rejected: obstacle detected")
            self.publish_done(False)
            return

        if self.current_distance + meters > self.MAX_TRAVEL_DISTANCE:
            meters = self.MAX_TRAVEL_DISTANCE - self.current_distance
            self.get_logger().warn(
                f"Clamped distance to {meters:.3f} m"
            )

        if meters <= 0.0:
            self.publish_done(True)
            return

        self.start_distance = self.current_distance
        self.target_delta = meters
        self.moving = True
        self._move_start_time = self.get_clock().now()

        self.get_logger().info(
            f"Moving {meters:.3f} m"
        )

    # ==================================================
    # Control Loop
    # ==================================================

    def control_loop(self):

        if not self.moving:
            return

        if not self._obstacle_sensor_alive():
            self.get_logger().error("EMERGENCY STOP: sensor timeout")
            self.abort_motion(False)
            return

        if self.obstacle_distance < self.OBSTACLE_THRESHOLD:
            self.get_logger().warn("EMERGENCY STOP: obstacle detected")
            self.abort_motion(False)
            return

        if self.current_distance >= self.MAX_TRAVEL_DISTANCE:
            self.get_logger().warn("STOP: max planter distance reached")
            self.abort_motion(False)
            return

        elapsed = (self.get_clock().now() - self._move_start_time).nanoseconds / 1e9
        if elapsed > self.MOVE_TIMEOUT:
            self.get_logger().error("Move timeout")
            self.abort_motion(False)
            return

        travelled = self.current_distance - self.start_distance
        error = self.target_delta - travelled

        if error <= self.distance_tolerance:
            self.get_logger().info("Target reached")
            self.abort_motion(True)
            return

        speed = self._compute_speed(error)
        self.publish_velocity(speed)

    # ==================================================
    # Helpers
    # ==================================================

    def _compute_speed(self, error):

        if error >= self.SLOWDOWN_ZONE:
            return self.linear_speed

        ratio = error / self.SLOWDOWN_ZONE
        speed = self.MIN_SPEED + ratio * (self.linear_speed - self.MIN_SPEED)
        return max(speed, self.MIN_SPEED)

    def _obstacle_sensor_alive(self):

        if self.obstacle_last_update is None:
            return False

        elapsed = (
            self.get_clock().now() - self.obstacle_last_update
        ).nanoseconds / 1e9

        return elapsed < self.OBSTACLE_TIMEOUT

    def publish_velocity(self, linear_x):

        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)

    def abort_motion(self, success):

        self.publish_velocity(0.0)
        self.moving = False
        self._move_start_time = None
        self.publish_done(success)

    def publish_done(self, success):

        msg = Bool()
        msg.data = success
        self.done_pub.publish(msg)
        self.get_logger().info(f"Move done — success={success}")

    def _publish_reset(self):

        msg = Bool()
        msg.data = True
        self.reset_pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)
    node = MovementNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()