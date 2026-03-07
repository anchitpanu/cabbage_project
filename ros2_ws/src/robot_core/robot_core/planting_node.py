import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32


class PlantNode(Node):

    def __init__(self):
        super().__init__('plant_node')

        # ---------- Plant sequence ----------
        # self.sequence = [
        #     5,  # stepper down
        #     1,  # gripper open
        #     6,  # stepper up
        #     2,  # gripper close
        #     3,  # feeder open
        #     5,  # stepper down
        #     1,  # gripper open
        #     6,  # stepper up
        #     2,  # gripper close
        #     4   # feeder close
        # ]

        # test basic planting
        self.sequence = [
            5,  # stepper down
            1,  # gripper open
            6,  # stepper up
            2,  # gripper close
        ]

        self.step_index = 0
        self.busy = False

        # ---------- Publishers ----------
        self.actuator_pub = self.create_publisher(
            Int32,
            '/microros/actuator_cmd',
            10
        )

        self.done_pub = self.create_publisher(
            Bool,
            '/plant_done',
            10
        )

        # ---------- Subscribers ----------
        self.create_subscription(
            Bool,
            '/plant_cmd',
            self.plant_cmd_callback,
            10
        )

        self.create_subscription(
            Bool,
            '/microros/actuator_ack',
            self.actuator_ack_callback,
            10
        )

        self.create_subscription(
            Int32,
            '/microros/actuator_error',
            self.actuator_error_callback,
            10
        )

        self.get_logger().info("Plant Node Ready")

    # ==================================================
    # Mission trigger
    # ==================================================

    def plant_cmd_callback(self, msg):

        if not msg.data:
            return

        if self.busy:
            self.get_logger().warn("Plant sequence already running")
            return

        self.busy = True
        self.step_index = 0

        self.get_logger().info("Starting planting sequence")

        self.send_next_command()

    # ==================================================
    # Send actuator command
    # ==================================================

    def send_next_command(self):

        if self.step_index >= len(self.sequence):

            self.get_logger().info("Plant sequence finished")

            msg = Bool()
            msg.data = True
            self.done_pub.publish(msg)

            self.busy = False
            return

        cmd = self.sequence[self.step_index]

        msg = Int32()
        msg.data = cmd

        self.actuator_pub.publish(msg)

        self.get_logger().info(f"Sent actuator command {cmd}")

    # ==================================================
    # ESP32 ACK
    # ==================================================

    def actuator_ack_callback(self, msg):

        if not self.busy:
            return

        if not msg.data:
            self.get_logger().error("Actuator reported failure")

            done = Bool()
            done.data = False
            self.done_pub.publish(done)

            self.busy = False
            return

        self.get_logger().info("Actuator step complete")

        self.step_index += 1

        self.send_next_command()

    # ==================================================
    # ESP32 ERROR
    # ==================================================

    def actuator_error_callback(self, msg):

        if not self.busy:
            return

        self.get_logger().error(f"Actuator error code: {msg.data}")

        done = Bool()
        done.data = False
        self.done_pub.publish(done)

        self.busy = False


# ======================================================
# Main
# ======================================================

def main(args=None):
    rclpy.init(args=args)
    node = PlantNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()