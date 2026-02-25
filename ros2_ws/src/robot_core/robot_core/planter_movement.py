import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool

class PlanterMovement(Node):

    def __init__(self):
        super().__init__('planter_movement')

        # ---- Parameters ----
        self.PLANTER_LENGTH = 1.50
        self.ENTRY_MARGIN = 0.30
        self.STOP_DISTANCE = self.PLANTER_LENGTH + self.ENTRY_MARGIN
        self.OBSTACLE_THRESHOLD = 0.15

        # ---- Internal State ----
        self.distance = 0.0
        self.vision_end = False
        self.obstacle_distance = 999.0
        self.stopped = False

        # ---- Subscribers ----
        self.create_subscription(
            Float32,
            '/distance_inside_planter',
            self.distance_callback,
            10)

        self.create_subscription(
            Bool,
            '/vision_end_detected',
            self.vision_callback,
            10)

        self.create_subscription(
            Float32,
            '/obstacle_distance',
            self.obstacle_callback,
            10)

        # ---- Publisher ----
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)

        self.timer = self.create_timer(0.05, self.control_loop)

    # ------------------------
    # Callbacks
    # ------------------------

    def distance_callback(self, msg):
        self.distance = msg.data

    def vision_callback(self, msg):
        self.vision_end = msg.data

    def obstacle_callback(self, msg):
        self.obstacle_distance = msg.data

    # ------------------------
    # Main Control
    # ------------------------

    def control_loop(self):

        cmd = Twist()

        # 🔴 Priority 1: Obstacle
        if self.obstacle_distance < self.OBSTACLE_THRESHOLD:
            self.get_logger().warn("EMERGENCY STOP: Obstacle")
            self.stopped = True

        # 🟢 Priority 2: Vision end detection
        elif self.vision_end:
            self.get_logger().info("STOP: Vision End Detected")
            self.stopped = True

        # 🟡 Priority 3: Encoder safety
        elif self.distance >= self.STOP_DISTANCE:
            self.get_logger().warn("STOP: Encoder Safety")
            self.stopped = True

        # 🟢 Normal movement
        else:
            cmd.linear.x = 0.20
            cmd.angular.z = 0.0
            self.stopped = False

        if self.stopped:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PlanterMovement()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()