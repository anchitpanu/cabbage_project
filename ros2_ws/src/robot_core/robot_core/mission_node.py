import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Bool, Float32
from enum import Enum, auto


class State(Enum):
    IDLE        = auto()
    ENTERING    = auto()    # waiting for EntryNavigation to finish
    MOVING      = auto()
    PLANTING    = auto()
    DONE        = auto()
    ABORTED     = auto()


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
        self.mission_step = 0
        self.cabbage_index = 0
        self.CABBAGE_COUNT = 5

        # ---------------- Timeout Settings ----------------
        self.PLANT_TIMEOUT  = 10.0
        self.MOVE_TIMEOUT   = 60.0
        self.ENTRY_TIMEOUT  = 30.0   # seconds to wait for entry to complete

        self._plant_timer   = None
        self._move_timer    = None
        self._entry_timer   = None   # watchdog for entry phase

        # ---------------- Stale Message Guard ----------------
        self._expecting_move_done  = False
        self._expecting_plant_done = False
        self._expecting_entry_done = False

        # ---------------- Publishers ----------------
        self.move_cmd_pub = self.create_publisher(
            Float32,
            '/quin/move_command',
            10
        )

        self.plant_pub = self.create_publisher(
            Bool,
            '/plant_cmd',
            10
        )

        self.mission_status_pub = self.create_publisher(
            Bool,
            '/quin/mission_status',
            10
        )

        # Triggers EntryNavigation to start
        self.entry_trigger_pub = self.create_publisher(
            Bool,
            '/entry_start',
            10
        )

        # ---------------- Subscribers ----------------
        self.create_subscription(
            Int32MultiArray,
            '/mission_params',
            self.mission_param_callback,
            10
        )

        self.create_subscription(
            Bool,
            '/quin/move_done',
            self.move_done_callback,
            10
        )

        self.create_subscription(
            Bool,
            '/plant_done',
            self.plant_done_callback,
            10
        )

        # Receives completion signal from EntryNavigation
        self.create_subscription(
            Bool,
            '/entry_done',
            self.entry_done_callback,
            10
        )

        # Allows external node to restart the mission
        self.create_subscription(
            Bool,
            '/quin/mission_restart',
            self.mission_restart_callback,
            10
        )

        self.get_logger().info("Mission Node Ready — waiting for AprilTag parameters...")

    # ==================================================
    # Parameter Reception
    # ==================================================

    def mission_param_callback(self, msg):
        """
        Step 1: AprilTag parameters received.
        Stores parameters then immediately triggers entry phase.
        """
        if self.state not in (State.IDLE, State.DONE, State.ABORTED):
            self.get_logger().warn(
                "Parameter update ignored — mission is currently running"
            )
            return

        self.AB = msg.data[0] / 100.0
        self.C  = msg.data[1] / 100.0
        self.DE = msg.data[2] / 100.0
        self.params_received = True

        self.get_logger().info(
            f"Parameters loaded → AB={self.AB:.3f} m  C={self.C:.3f} m  DE={self.DE:.3f} m"
        )

        # Step 2: trigger entry phase before any movement/planting
        self._start_entry_phase()

    # ==================================================
    # Entry Phase
    # ==================================================

    def _start_entry_phase(self):
        """
        Step 2: Triggers EntryNavigation node to align and enter the planter.
        Waits for /entry_done before starting the planting mission.
        """
        self.get_logger().info("Triggering entry phase — waiting for /entry_done...")
        self.state = State.ENTERING
        self._expecting_entry_done = True

        # Signal EntryNavigation to begin
        msg = Bool()
        msg.data = True
        self.entry_trigger_pub.publish(msg)

        # Watchdog in case EntryNavigation never responds
        self._entry_timer = self.create_timer(
            self.ENTRY_TIMEOUT,
            self._entry_timeout_cb
        )

    def entry_done_callback(self, msg: Bool):
        """
        Step 3: Called when EntryNavigation completes.
        If successful, starts the planting mission.
        """
        if not self._expecting_entry_done:
            self.get_logger().warn("Stale /entry_done received — ignored")
            return

        self._expecting_entry_done = False
        self._cancel_timer(self._entry_timer)
        self._entry_timer = None

        if not msg.data:
            self._abort("Entry phase failed")
            return

        self.get_logger().info("Entry complete — starting planting mission")
        self._reset_mission_steps()
        self.advance_mission()

    def _entry_timeout_cb(self):
        self.get_logger().error(
            f"TIMEOUT: No /entry_done received after {self.ENTRY_TIMEOUT}s"
        )
        self._cancel_timer(self._entry_timer)
        self._entry_timer = None
        self._expecting_entry_done = False
        self._abort("Entry phase timeout")

    # ==================================================
    # Mission Restart
    # ==================================================

    def mission_restart_callback(self, msg: Bool):
        if not msg.data:
            return

        if self.state in (State.MOVING, State.PLANTING, State.ENTERING):
            self.get_logger().warn("Restart ignored — mission is currently running")
            return

        if not self.params_received:
            self.get_logger().warn("Restart ignored — no parameters loaded yet")
            return

        self.get_logger().info("Mission restart — re-running entry phase")
        self._cancel_all_timers()
        self._reset_all_state()
        self._start_entry_phase()

    def _reset_all_state(self):
        self.state = State.IDLE
        self._reset_mission_steps()
        self._expecting_move_done  = False
        self._expecting_plant_done = False
        self._expecting_entry_done = False

    def _reset_mission_steps(self):
        self.mission_step  = 0
        self.cabbage_index = 0

    # ==================================================
    # State Machine — Advance
    # ==================================================

    def advance_mission(self):
        """
        Only called AFTER entry phase succeeds.

        Step 0  → Move AB (seed 1 position)
        Step 1  → Plant seed 1
        Step 2  → Move AB (seed 2 position)
        Step 3  → Plant seed 2
        Step 4  → Move transition distance
        Step 5+ → Cabbage loop: move DE (repeats CABBAGE_COUNT times)
        Final   → Mission complete
        """
        if not self.params_received:
            self.get_logger().error("Cannot advance — parameters not loaded")
            self._abort("Parameters missing")
            return

        step = self.mission_step
        self.get_logger().info(f"Mission step {step}")

        if step == 0:
            self.send_move(self.AB)

        elif step == 1:
            self.do_plant(seed_number=1)

        elif step == 2:
            self.send_move(self.AB)

        elif step == 3:
            self.do_plant(seed_number=2)

        elif step == 4:
            extra_offset = 0.30
            transition = self.C + extra_offset
            self.send_move(transition)

        elif step >= 5:
            if self.cabbage_index < self.CABBAGE_COUNT:
                self.get_logger().info(
                    f"Cabbage interval {self.cabbage_index + 1}/{self.CABBAGE_COUNT}"
                )
                self.cabbage_index += 1
                self.send_move(self.DE)
                return

            else:
                self.state = State.DONE
                self.get_logger().info("✓ Mission Complete")
                self._publish_mission_status(success=True)
                return

        self.mission_step += 1

    # ==================================================
    # Movement
    # ==================================================

    def send_move(self, meters: float):
        self.state = State.MOVING
        self._expecting_move_done = True

        msg = Float32()
        msg.data = float(meters)
        self.move_cmd_pub.publish(msg)
        self.get_logger().info(f"Sent move command: {meters:.3f} m")

        self._move_timer = self.create_timer(
            self.MOVE_TIMEOUT,
            self._move_timeout_cb
        )

    def move_done_callback(self, msg: Bool):
        if not self._expecting_move_done:
            self.get_logger().warn("Stale /move_done received — ignored")
            return

        self._expecting_move_done = False
        self._cancel_timer(self._move_timer)
        self._move_timer = None

        if not msg.data:
            self._abort("Movement failed")
            return

        self.get_logger().info("Move confirmed done")
        self.advance_mission()

    def _move_timeout_cb(self):
        self.get_logger().error(
            f"TIMEOUT: No /move_done received after {self.MOVE_TIMEOUT}s"
        )
        self._cancel_timer(self._move_timer)
        self._move_timer = None
        self._expecting_move_done = False
        self._abort("Movement timeout")

    # ==================================================
    # Planting
    # ==================================================

    def do_plant(self, seed_number: int):
        self.state = State.PLANTING
        self._expecting_plant_done = True

        self.get_logger().info(f"Planting seed {seed_number}")
        msg = Bool()
        msg.data = True
        self.plant_pub.publish(msg)

        self._plant_timer = self.create_timer(
            self.PLANT_TIMEOUT,
            self._plant_timeout_cb
        )

    def plant_done_callback(self, msg: Bool):
        if not self._expecting_plant_done:
            self.get_logger().warn("Stale /plant_done received — ignored")
            return

        self._expecting_plant_done = False
        self._cancel_timer(self._plant_timer)
        self._plant_timer = None

        if not msg.data:
            self._abort("Planting failed")
            return

        self.get_logger().info("Planting confirmed done")
        self.advance_mission()

    def _plant_timeout_cb(self):
        self.get_logger().error(
            f"TIMEOUT: No /plant_done received after {self.PLANT_TIMEOUT}s"
        )
        self._cancel_timer(self._plant_timer)
        self._plant_timer = None
        self._expecting_plant_done = False
        self._abort("Planting timeout")

    # ==================================================
    # Abort
    # ==================================================

    def _abort(self, reason: str):
        self.get_logger().error(f"Mission ABORTED — reason: {reason}")
        self.state = State.ABORTED
        self._cancel_all_timers()
        self._expecting_move_done  = False
        self._expecting_plant_done = False
        self._expecting_entry_done = False
        self._publish_mission_status(success=False)

    # ==================================================
    # Helpers
    # ==================================================

    def _cancel_timer(self, timer_handle):
        if timer_handle is not None:
            timer_handle.cancel()

    def _cancel_all_timers(self):
        self._cancel_timer(self._plant_timer)
        self._cancel_timer(self._move_timer)
        self._cancel_timer(self._entry_timer)
        self._plant_timer  = None
        self._move_timer   = None
        self._entry_timer  = None

    def _publish_mission_status(self, success: bool):
        msg = Bool()
        msg.data = success
        self.mission_status_pub.publish(msg)


# ======================================================
# Main
# ======================================================

def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()