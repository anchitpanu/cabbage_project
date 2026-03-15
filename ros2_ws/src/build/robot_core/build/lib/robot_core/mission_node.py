import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Bool, Float32, String
from rclpy.qos import QoSProfile, ReliabilityPolicy
from enum import Enum, auto
from datetime import datetime


class State(Enum):
    IDLE        = auto()
    ENTERING    = auto()
    RESETTING   = auto()
    MOVING      = auto()
    PLANTING    = auto()
    DETECTING   = auto()
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
        self.mission_step   = 0
        self.cabbage_index  = 0

        # ---------------- Cabbage Zone Distance Tracking ----------------
        self.CABBAGE_ZONE_MAX_CM   = 170.0
        self.cabbage_cumulative_cm = 0.0

        # ---------------- Cabbage Log ----------------
        self.cabbage_log = []

        # ---------------- Robot Ready Flag ----------------
        self._robot_ready = False

        # ---------------- Timeout Settings ----------------
        self.PLANT_TIMEOUT   = 60.0
        self.MOVE_TIMEOUT    = 60.0
        self.ENTRY_TIMEOUT   = 60.0
        self.RESET_TIMEOUT   = 10.0
        self.DETECT_TIMEOUT  = 15.0

        self._plant_timer    = None
        self._move_timer     = None
        self._entry_timer    = None
        self._reset_timer    = None
        self._detect_timer   = None

        # ---------------- Stale Message Guards ----------------
        self._expecting_move_done    = False
        self._expecting_plant_done   = False
        self._expecting_entry_done   = False
        self._expecting_robot_ready  = False
        self._expecting_detect_done  = False

        # ---------------- QoS ----------------
        esp_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        # ---------------- Publishers ----------------
        self.move_cmd_pub = self.create_publisher(
            Float32, '/quin/cmd_move', 10)

        self.plant_pub = self.create_publisher(
            Bool, '/plant_cmd', 10)

        self.mission_status_pub = self.create_publisher(
            Bool, '/quin/mission_status', 10)

        self.entry_trigger_pub = self.create_publisher(
            Bool, '/entry_start', 10)

        self.reset_trigger_pub = self.create_publisher(
            Bool, '/quin/trigger_reset', 10)

        self.detect_trigger_pub = self.create_publisher(
            Bool, '/quin/detect_trigger', 10)

        self.cabbage_log_pub = self.create_publisher(
            String, '/quin/cabbage_log', 10)

        # ---------------- Subscribers ----------------
        self.create_subscription(
            Int32MultiArray, '/mission_params',
            self.mission_param_callback, 10)

        self.create_subscription(
            Bool, '/quin/move_done',
            self.move_done_callback, 10)

        self.create_subscription(
            Bool, '/plant_done',
            self.plant_done_callback, 10)

        self.create_subscription(
            Bool, '/entry_done',
            self.entry_done_callback, 10)

        self.create_subscription(
            Bool, '/quin/mission_restart',
            self.mission_restart_callback, 10)

        # Subscribe โดยตรงจาก ESP32 ด้วย BEST_EFFORT
        self.create_subscription(
            Bool, '/quin/robot_ready',
            self.robot_ready_callback, esp_qos)

        self.create_subscription(
            String, '/quin/detect_result',
            self.detect_result_callback, 10)

        self.get_logger().info("Mission Node Ready — waiting for AprilTag parameters...")

    # ==================================================
    # Parameter Reception
    # ==================================================

    def mission_param_callback(self, msg):
        if self.state not in (State.IDLE, State.DONE, State.ABORTED):
            self.get_logger().warn("Parameter update ignored — mission is currently running")
            return

        self.AB = msg.data[0] / 100.0
        self.C  = msg.data[1] / 100.0
        self.DE = msg.data[2] / 100.0
        self.params_received = True

        self.get_logger().info(
            f"Parameters loaded → AB={self.AB:.3f} m  C={self.C:.3f} m  DE={self.DE:.3f} m"
        )

        # self._start_entry_phase()

        self._trigger_reset()        

    # ==================================================
    # Entry Phase
    # ==================================================

    def _start_entry_phase(self):
        self.get_logger().info("Triggering entry phase — waiting for /entry_done...")
        self.state = State.ENTERING
        self._expecting_entry_done = True

        msg = Bool()
        msg.data = True
        self.entry_trigger_pub.publish(msg)

        self._entry_timer = self.create_timer(self.ENTRY_TIMEOUT, self._entry_timeout_cb)

    def entry_done_callback(self, msg: Bool):
        if not self._expecting_entry_done:
            self.get_logger().warn("Stale /entry_done received — ignored")
            return

        self._expecting_entry_done = False
        self._cancel_timer(self._entry_timer)
        self._entry_timer = None

        if not msg.data:
            self._abort("Entry phase failed")
            return

        self.get_logger().info("Entry complete — triggering encoder reset before mission")
        self._trigger_reset()

    def _entry_timeout_cb(self):
        self.get_logger().error(f"TIMEOUT: No /entry_done received after {self.ENTRY_TIMEOUT}s")
        self._cancel_timer(self._entry_timer)
        self._entry_timer = None
        self._expecting_entry_done = False
        self._abort("Entry phase timeout")

    # ==================================================
    # Encoder Reset
    # ==================================================

    def _trigger_reset(self):
        self.get_logger().info("Triggering encoder reset — waiting for confirmation...")
        self.state = State.RESETTING
        self._robot_ready            = False
        self._expecting_robot_ready  = True
        self._expecting_plant_done   = False
        self._expecting_move_done    = False
        self._expecting_detect_done  = False

        msg = Bool()
        msg.data = True
        self.reset_trigger_pub.publish(msg)

        self._reset_timer = self.create_timer(self.RESET_TIMEOUT, self._reset_timeout_cb)

    def robot_ready_callback(self, msg: Bool):
        if not self._expecting_robot_ready:
            self.get_logger().warn("Stale /quin/robot_ready received — ignored")
            return

        if not msg.data:
            return

        self._expecting_robot_ready = False
        self._cancel_timer(self._reset_timer)
        self._reset_timer = None

        self._robot_ready = True
        self.get_logger().info("Robot ready — encoder confirmed reset, starting mission")
        self._reset_mission_steps()
        self.advance_mission()

    def _reset_timeout_cb(self):
        self.get_logger().error(f"TIMEOUT: No /quin/robot_ready received after {self.RESET_TIMEOUT}s")
        self._cancel_timer(self._reset_timer)
        self._reset_timer = None
        self._expecting_robot_ready = False
        self._abort("Encoder reset timeout")

    # ==================================================
    # Mission Restart
    # ==================================================

    def mission_restart_callback(self, msg: Bool):
        if not msg.data:
            return

        if self.state in (State.MOVING, State.PLANTING, State.ENTERING,
                          State.RESETTING, State.DETECTING):
            self.get_logger().warn("Restart ignored — mission is currently running")
            return

        if not self.params_received:
            self.get_logger().warn("Restart ignored — no parameters loaded yet")
            return

        self.get_logger().info("Mission restart — re-entering planting box")
        self._cancel_all_timers()
        self._reset_all_state()
        self._start_entry_phase()

    def _reset_all_state(self):
        self.state = State.IDLE
        self._reset_mission_steps()
        self._robot_ready            = False
        self._expecting_move_done    = False
        self._expecting_plant_done   = False
        self._expecting_entry_done   = False
        self._expecting_robot_ready  = False
        self._expecting_detect_done  = False

    def _reset_mission_steps(self):
        self.mission_step          = 0
        self.cabbage_index         = 0
        self.cabbage_cumulative_cm = 0.0
        self.cabbage_log           = []

    # ==================================================
    # State Machine — Advance
    # ==================================================

    def advance_mission(self):
        if not self.params_received:
            self._abort("Parameters missing")
            return

        if not self._robot_ready:
            self._abort("Robot not ready")
            return

        step = self.mission_step
        self.get_logger().info(f"Mission step {step}")

        if step == 0:
            self.mission_step += 1
            self.send_move(self.AB)

        elif step == 1:
            self.mission_step += 1
            self.do_plant(seed_number=1)

        elif step == 2:
            self.mission_step += 1
            self.send_move(self.AB)

        elif step == 3:
            self.mission_step += 1
            self.do_plant(seed_number=2)

        elif step == 4:
            self.mission_step += 1
            self.send_move(self.C + 0.30)

        elif step >= 5:
            if self.cabbage_cumulative_cm < self.CABBAGE_ZONE_MAX_CM:
                self.cabbage_index += 1
                self.get_logger().info(
                    f"Cabbage {self.cabbage_index} — "
                    f"cumulative {self.cabbage_cumulative_cm:.1f} / {self.CABBAGE_ZONE_MAX_CM} cm"
                )
                self.do_detect()
            else:
                self._save_log_file()
                self.state = State.DONE
                self.get_logger().info("Mission Complete")
                self._publish_mission_status(success=True)

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

        self._move_timer = self.create_timer(self.MOVE_TIMEOUT, self._move_timeout_cb)

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

        if self.mission_step >= 5:
            self.cabbage_cumulative_cm += self.DE * 100.0
            self.get_logger().info(
                f"Cabbage zone distance: {self.cabbage_cumulative_cm:.1f} / "
                f"{self.CABBAGE_ZONE_MAX_CM} cm"
            )

        self.advance_mission()

    def _move_timeout_cb(self):
        self.get_logger().error(f"TIMEOUT: No /move_done received after {self.MOVE_TIMEOUT}s")
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

        self._plant_timer = self.create_timer(self.PLANT_TIMEOUT, self._plant_timeout_cb)

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
        self.get_logger().error(f"TIMEOUT: No /plant_done received after {self.PLANT_TIMEOUT}s")
        self._cancel_timer(self._plant_timer)
        self._plant_timer = None
        self._expecting_plant_done = False
        self._abort("Planting timeout")

    # ==================================================
    # Cabbage Detection
    # ==================================================

    def do_detect(self):
        self.state = State.DETECTING
        self._expecting_detect_done = True

        self.get_logger().info(f"Triggering cabbage detection #{self.cabbage_index}...")
        msg = Bool()
        msg.data = True
        self.detect_trigger_pub.publish(msg)

        self._detect_timer = self.create_timer(self.DETECT_TIMEOUT, self._detect_timeout_cb)

    def detect_result_callback(self, msg: String):
        if not self._expecting_detect_done:
            self.get_logger().warn("Stale /quin/detect_result received — ignored")
            return

        self._expecting_detect_done = False
        self._cancel_timer(self._detect_timer)
        self._detect_timer = None

        try:
            parts       = msg.data.split(',')
            size_cm     = float(parts[0])
            harvestable = int(parts[1])
        except Exception:
            self.get_logger().error(f"Bad detect_result format: '{msg.data}' — skipping")
            size_cm     = 0.0
            harvestable = -1

        if harvestable == 1:
            status_str = "Harvestable"
        elif harvestable == 0:
            status_str = "Not harvestable"
        else:
            status_str = "Not detected"

        entry = {
            'index':       self.cabbage_index,
            'size_cm':     round(size_cm, 1),
            'harvestable': harvestable,
            'status':      status_str,
        }
        self.cabbage_log.append(entry)

        self.get_logger().info(
            f"Cabbage {self.cabbage_index}: {size_cm:.1f} cm — {status_str}"
        )

        self._publish_cabbage_log()
        self.send_move(self.DE)

    def _detect_timeout_cb(self):
        self.get_logger().error(
            f"TIMEOUT: No /quin/detect_result received after {self.DETECT_TIMEOUT}s"
        )
        self._cancel_timer(self._detect_timer)
        self._detect_timer = None
        self._expecting_detect_done = False

        entry = {
            'index':       self.cabbage_index,
            'size_cm':     0.0,
            'harvestable': -1,
            'status':      'Timeout — not detected',
        }
        self.cabbage_log.append(entry)
        self.get_logger().warn(
            f"Cabbage {self.cabbage_index} timed out — logged as not detected, continuing"
        )
        self._publish_cabbage_log()
        self.send_move(self.DE)

    # ==================================================
    # Cabbage Log
    # ==================================================

    def _publish_cabbage_log(self):
        import json
        msg      = String()
        msg.data = json.dumps(self.cabbage_log)
        self.cabbage_log_pub.publish(msg)

    def _save_log_file(self):
        timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        filename  = f'/home/quin/cabbage_project/logs/mission_{timestamp}.txt'
        os.makedirs(os.path.dirname(filename), exist_ok=True)

        harvestable_count = sum(1 for e in self.cabbage_log if e['harvestable'] == 1)
        not_ready_count   = sum(1 for e in self.cabbage_log if e['harvestable'] == 0)
        no_detect_count   = sum(1 for e in self.cabbage_log if e['harvestable'] == -1)

        lines = [
            f"Mission Log — {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
            f"Parameters  — AB={self.AB:.3f} m  C={self.C:.3f} m  DE={self.DE:.3f} m",
            "=" * 50,
        ]
        for e in self.cabbage_log:
            lines.append(
                f"Cabbage {e['index']:>2}:  {e['size_cm']:>5.1f} cm  —  {e['status']}"
            )
        lines += [
            "=" * 50,
            f"Total scanned  : {len(self.cabbage_log)}",
            f"Harvestable    : {harvestable_count}",
            f"Not ready      : {not_ready_count}",
            f"Not detected   : {no_detect_count}",
        ]

        with open(filename, 'w', encoding='utf-8') as f:
            f.write('\n'.join(lines) + '\n')

        self.get_logger().info(f"Log saved → {filename}")

    # ==================================================
    # Abort
    # ==================================================

    def _abort(self, reason: str):
        self.get_logger().error(f"Mission ABORTED — reason: {reason}")
        self.state = State.ABORTED
        self._cancel_all_timers()
        self._expecting_move_done    = False
        self._expecting_plant_done   = False
        self._expecting_entry_done   = False
        self._expecting_robot_ready  = False
        self._expecting_detect_done  = False

        if self.cabbage_log:
            self.get_logger().info("Saving partial cabbage log before abort...")
            self._save_log_file()

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
        self._cancel_timer(self._reset_timer)
        self._cancel_timer(self._detect_timer)
        self._plant_timer   = None
        self._move_timer    = None
        self._entry_timer   = None
        self._reset_timer   = None
        self._detect_timer  = None

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