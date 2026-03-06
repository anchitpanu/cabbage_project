import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Bool, Float32
from robot_core.entry_controller import EntryController
from enum import Enum, auto


class State(Enum):
    IDLE            = auto()
    ENTERING        = auto()
    MOVING          = auto()
    PLANTING        = auto()
    DONE            = auto()
    ABORTED         = auto()


class MissionNode(Node):

    def __init__(self):
        super().__init__('mission_node')

        # ---------------- Mission Parameters ----------------
        self.AB = None
        self.C  = None
        self.DE = None
        self.params_received = False

        # ---------------- State Machine ----------------
        self.state = State.IDLE
        self.mission_step = 0       # tracks which step we're on
        self.cabbage_index = 0      # tracks cabbage loop iteration
        self.CABBAGE_COUNT = 5

        # ---------------- Entry Controller ----------------
        self.entry_controller = EntryController()

        # ---------------- Publishers ----------------

        # Sends move distance commands to MovementNode
        self.move_cmd_pub = self.create_publisher(
            Float32,
            '/quin/move_command',
            10
        )

        # Triggers the planter hardware
        self.plant_pub = self.create_publisher(
            Bool,
            '/plant_cmd',
            10
        )

        # ---------------- Subscribers ----------------

        # Receives AprilTag parameters
        self.create_subscription(
            Int32MultiArray,
            '/mission_params',
            self.mission_param_callback,
            10
        )

        # Receives movement completion signal from MovementNode
        self.create_subscription(
            Bool,
            '/quin/move_done',
            self.move_done_callback,
            10
        )

        # Receives planting completion signal from planter hardware
        self.create_subscription(
            Bool,
            '/plant_done',
            self.plant_done_callback,
            10
        )

        self.get_logger().info("Mission Node Ready — waiting for AprilTag parameters...")

    # ==================================================
    # Parameter Reception
    # ==================================================

    def mission_param_callback(self, msg):
        if self.params_received:
            return  # Ignore duplicate messages

        self.AB = msg.data[0] / 100.0
        self.C  = msg.data[1] / 100.0
        self.DE = msg.data[2] / 100.0

        self.params_received = True
        self.get_logger().info(
            f"Parameters loaded → AB={self.AB:.3f} m  C={self.C:.3f} m  DE={self.DE:.3f} m"
        )

        self.start_entry_phase()

    # ==================================================
    # Entry Phase
    # ==================================================

    def start_entry_phase(self):
        self.get_logger().info("Entry phase: aligning with planter...")
        self.state = State.ENTERING

        # EntryController is synchronous (hardware alignment step)
        # If it becomes async in future, convert this to a topic-based handshake too
        self.entry_controller.execute_entry()

        self.get_logger().info("Entry complete — starting mission")
        self.mission_step = 0
        self.advance_mission()

    # ==================================================
    # State Machine — Advance
    # ==================================================

    def advance_mission(self):
        """
        Called after each movement or planting step completes.
        Each step is numbered so the flow is easy to follow and extend.

        Step 0  → Move AB (to seed 1 position)
        Step 1  → Plant seed 1
        Step 2  → Move AB (to seed 2 position)
        Step 3  → Plant seed 2
        Step 4  → Move transition distance
        Step 5+ → Cabbage loop: move DE, log detection (repeats CABBAGE_COUNT times)
        Final   → Mission complete
        """

        step = self.mission_step
        self.get_logger().info(f"Mission step {step}")

        # ---- Seed 1 ----
        if step == 0:
            self.send_move(self.AB)

        elif step == 1:
            self.do_plant(seed_number=1)

        # ---- Seed 2 ----
        elif step == 2:
            self.send_move(self.AB)

        elif step == 3:
            self.do_plant(seed_number=2)

        # ---- Transition ----
        elif step == 4:
            extra_offset = 0.30  # front-to-back robot offset in meters
            transition = self.C + extra_offset
            self.send_move(transition)

        # ---- Cabbage Loop ----
        elif step >= 5:
            if self.cabbage_index < self.CABBAGE_COUNT:
                self.get_logger().info(
                    f"Cabbage interval {self.cabbage_index + 1}/{self.CABBAGE_COUNT}"
                )
                self.cabbage_index += 1
                self.send_move(self.DE)
                return  # Don't increment mission_step — loop stays at this step

            else:
                # All cabbage intervals done
                self.state = State.DONE
                self.get_logger().info("✓ Mission Complete")
                return

        self.mission_step += 1

    # ==================================================
    # Movement
    # ==================================================

    def send_move(self, meters: float):
        self.state = State.MOVING
        msg = Float32()
        msg.data = float(meters)
        self.move_cmd_pub.publish(msg)
        self.get_logger().info(f"Sent move command: {meters:.3f} m")

    def move_done_callback(self, msg: Bool):
        """Called by MovementNode when a move finishes."""
        if self.state != State.MOVING:
            return  # Stale message, ignore

        if not msg.data:
            # Movement was aborted (obstacle or safety limit)
            self.get_logger().error("Movement aborted — mission halted")
            self.state = State.ABORTED
            return

        self.get_logger().info("Move confirmed done")
        self.advance_mission()

    # ==================================================
    # Planting
    # ==================================================

    def do_plant(self, seed_number: int):
        self.state = State.PLANTING
        self.get_logger().info(f"Planting seed {seed_number}")
        msg = Bool()
        msg.data = True
        self.plant_pub.publish(msg)
        # Waits for /plant_done topic instead of time.sleep()

    def plant_done_callback(self, msg: Bool):
        """Called by planter hardware when planting is complete."""
        if self.state != State.PLANTING:
            return

        if not msg.data:
            self.get_logger().error("Planting failed — mission halted")
            self.state = State.ABORTED
            return

        self.get_logger().info("Planting confirmed done")
        self.advance_mission()


# ======================================================
# Main
# ======================================================

def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()