import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Bool

from robot_core.entry_controller import EntryController
from robot_core.movement_node import MovementNode


class MissionNode(Node):

    def __init__(self):
        super().__init__('mission_node')

        # ---- Mission Parameters ----
        self.AB = None
        self.C = None
        self.DE = None

        self.params_received = False
        self.mission_started = False

        # ---- Movement Interface ----
        self.entry_controller = EntryController()
        self.movement = MovementNode()

        # ---- Plant Publisher ----
        self.plant_pub = self.create_publisher(
            Bool,
            '/plant_cmd',
            10
        )

        # ---- Subscribe to AprilTag Parameters ----
        self.create_subscription(
            Int32MultiArray,
            '/mission_params',
            self.mission_param_callback,
            10
        )

        self.get_logger().info("Waiting for AprilTag mission parameters...")

    # ------------------------------------------------
    # AprilTag Callback
    # ------------------------------------------------
    def mission_param_callback(self, msg):

        if self.params_received:
            return   # Ignore if mission already running

        AB_cm = msg.data[0]
        C_unit = msg.data[1]
        DE_cm = msg.data[2]

        self.AB = AB_cm / 100.0
        self.C  = C_unit / 100.0
        self.DE = DE_cm / 100.0

        self.params_received = True

        self.get_logger().info(
            f"Parameters loaded → AB = {self.AB} m, C = {self.C} m, DE = {self.DE} m"
        )

        # Start entry phase
        self.start_entry_phase()

    # ------------------------------------------------
    # Entry Phase
    # ------------------------------------------------

    def start_entry_phase(self):

        self.get_logger().info("Starting entry phase. Aligning with planter...")

        # Align and enter planter
        self.entry_controller.execute_entry()

        self.get_logger().info("Entry complete. Starting mission.")

        # Now start real mission
        self.start_mission()

    # ------------------------------------------------
    # Mission Execution
    # ------------------------------------------------
    def start_mission(self):

        if self.mission_started:
            return

        self.mission_started = True
        self.get_logger().info("Mission Started")

        # ---- Seed 1 ----
        self.movement.move_distance(self.AB)
        self.plant_once(1)

        # ---- Seed 2 ----
        self.movement.move_distance(self.AB)
        self.plant_once(2)

        # ---- Transition ----
        extra_offset = 30/100.0   # from front to back of robot, in meters
        transition_distance = (self.C * 5) + extra_offset

        self.movement.move_distance(transition_distance)

        # ---- Cabbage Detection Loop ----
        for i in range(5):   # example fixed count
            self.movement.move_distance(self.DE)
            self.get_logger().info(f"Cabbage interval {i}")

        self.get_logger().info("Mission Complete")

    # ------------------------------------------------
    # Planting
    # ------------------------------------------------
    def plant_once(self, seed_number):

        self.get_logger().info(f"Planting Seed {seed_number}")

        msg = Bool()
        msg.data = True
        self.plant_pub.publish(msg)

        time.sleep(1.0)


# ----------------------------------------------------
# Main
# ----------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()