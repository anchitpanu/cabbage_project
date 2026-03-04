import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import time


class MovementTester(Node):
    def __init__(self):
        super().__init__('movement_tester')

        # Publisher — matches /quin/cmd_move in firmware
        self.cmd_pub = self.create_publisher(Twist, '/quin/cmd_move', 10)

        # Reset distance publisher — matches /quin/reset_distance in firmware
        self.reset_pub = self.create_publisher(Twist, '/quin/reset_distance', 10)

        # Distance subscriber — feedback from firmware
        self.distance_sub = self.create_subscription(
            Float32, '/quin/distance_inside_planter',
            self.distance_callback, 10)

        # Debug subscribers — optional but useful
        self.motor_debug_sub = self.create_subscription(
            Twist, '/quin/debug/motor',
            self.motor_debug_callback, 10)

        self.encoder_debug_sub = self.create_subscription(
            Twist, '/quin/debug/encoder',
            self.encoder_debug_callback, 10)

        self.current_distance = 0.0

        self.get_logger().info("Waiting for firmware connection...")
        time.sleep(2)
        self.run_test()

    # -------- Callbacks --------

    def distance_callback(self, msg):
        self.current_distance = msg.data

    def motor_debug_callback(self, msg):
        # linear.y = left_rpm sign, linear.z = right_rpm sign
        self.get_logger().info(
            f"[MOTOR DEBUG] vx={msg.linear.x:.2f} wz={msg.angular.z:.2f} "
            f"left_rpm={msg.linear.y:.1f} right_rpm={msg.linear.z:.1f}"
        )

    def encoder_debug_callback(self, msg):
        # linear.x/y/z = d1/d2/d3, angular.z = d4
        self.get_logger().info(
            f"[ENCODER DEBUG] M1={msg.linear.x:.0f} M2={msg.linear.y:.0f} "
            f"M3={msg.linear.z:.0f} M4={msg.angular.z:.0f}"
        )

    # -------- Helpers --------

    def publish_cmd(self, vx, wz, duration):
        """Send a command for fixed duration, then stop."""
        self.get_logger().info(f"CMD: vx={vx} wz={wz} for {duration}s")
        t0 = time.time()
        while time.time() - t0 < duration:
            msg = Twist()
            msg.linear.x = float(vx)
            msg.angular.z = float(wz)
            self.cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.01)

        self.stop()
        time.sleep(0.5)

    def stop(self):
        self.cmd_pub.publish(Twist())

    def reset_distance(self):
        """Publish to /quin/reset_distance — firmware resets odometer on any message."""
        self.reset_pub.publish(Twist())
        self.current_distance = 0.0
        rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info("Distance reset")

    # -------- Test sequence --------

    def run_test(self):

        # --- TEST 1: Forward ---
        self.get_logger().info("=== TEST 1: Forward 2s ===")
        self.reset_distance()
        self.publish_cmd(0.2, 0.0, 2.0)
        self.get_logger().info(f"Distance traveled: {self.current_distance:.4f}m")
        time.sleep(1)

        # --- TEST 2: Backward ---
        self.get_logger().info("=== TEST 2: Backward 2s ===")
        self.reset_distance()
        self.publish_cmd(-0.2, 0.0, 2.0)
        self.get_logger().info(f"Distance traveled: {self.current_distance:.4f}m")
        time.sleep(1)

        # --- TEST 3: Turn Left ---
        # Firmware: Wz = -angular.z, so positive wz = left turn
        self.get_logger().info("=== TEST 3: Turn Left 2s ===")
        self.reset_distance()
        self.publish_cmd(0.0, 0.5, 2.0)
        self.get_logger().info(f"Distance after turn: {self.current_distance:.4f}m (should be ~0)")
        time.sleep(1)

        # --- TEST 4: Turn Right ---
        self.get_logger().info("=== TEST 4: Turn Right 2s ===")
        self.reset_distance()
        self.publish_cmd(0.0, -0.5, 2.0)
        self.get_logger().info(f"Distance after turn: {self.current_distance:.4f}m (should be ~0)")
        time.sleep(1)

        # --- TEST 5: CMD timeout ---
        # Firmware stops motors after 500ms with no message
        self.get_logger().info("=== TEST 5: CMD Timeout — stop publishing for 1s ===")
        time.sleep(1.0)
        self.get_logger().info("Motors should have stopped after 500ms")

        self.get_logger().info("=== All tests complete ===")


def main():
    rclpy.init()
    node = MovementTester()
    rclpy.shutdown()


if __name__ == '__main__':
    main()