import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool


class EntryNavigation(Node):

    def __init__(self):
        super().__init__('entry_navigation')

        # ---------------- Parameters ----------------
        self.TARGET_X            = 530.0   # pixel center target — adjust per camera mount
        self.ALIGN_TOLERANCE     = 5.0     # pixels — acceptable alignment error
        self.APPROACH_TOLERANCE  = 8.0     # pixels — acceptable error while moving
        self.CURVE_DURATION      = 2.5     # seconds for initial curve
        self.FINAL_FORWARD_TIME  = 1.6     # seconds of final straight drive
        self.SEARCH_TIMEOUT      = 10.0    # seconds before search aborts
        self.TAG_LOST_TIMEOUT    = 1.0     # seconds before tag considered lost

        # ---------------- Internal State ----------------
        self.center_x         = None
        self.tag_last_seen    = None       # ROS time of last tag message
        self.STATE            = "START_CURVE"
        self.curve_start      = self.get_clock().now()   # FIX: ROS clock
        self.final_start      = None
        self.search_start     = None
        self._done_published  = False      # FIX: publish done only once

        # ---------------- Publishers ----------------
        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel',     10)
        self.done_pub = self.create_publisher(Bool,  '/entry_done',  10)

        # ---------------- Subscribers ----------------
        self.create_subscription(
            Float32,
            '/tag_center_x',
            self.tag_callback,
            10
        )

        # ---------------- Timer ----------------
        self.timer = self.create_timer(0.05, self.loop)

        self.get_logger().info("Entry Navigation Ready")

    # ==================================================
    # Callbacks
    # ==================================================

    def tag_callback(self, msg):
        self.center_x      = msg.data
        self.tag_last_seen = self.get_clock().now()   # FIX: track last seen time

    # ==================================================
    # Helpers
    # ==================================================

    def send_cmd(self, linear, angular):
        msg = Twist()
        msg.linear.x  = linear
        msg.angular.z = angular
        self.cmd_pub.publish(msg)

    def stop(self):
        self.send_cmd(0.0, 0.0)

    def elapsed(self, since) -> float:
        """Returns seconds since a ROS Time stamp."""
        return (self.get_clock().now() - since).nanoseconds / 1e9

    def tag_is_visible(self) -> bool:
        """
        FIX: Returns True only if a tag message arrived recently.
        Prevents acting on stale center_x values.
        """
        if self.tag_last_seen is None:
            return False
        return self.elapsed(self.tag_last_seen) < self.TAG_LOST_TIMEOUT

    def transition(self, new_state: str):
        """FIX: Log every state transition for field debugging."""
        self.get_logger().info(f"State: {self.STATE} → {new_state}")
        self.STATE = new_state

    def abort(self, reason: str):
        self.stop()
        self.get_logger().error(f"Entry ABORTED — {reason}")
        self.transition("ABORTED")
        self._publish_done(success=False)

    def _publish_done(self, success: bool):
        # FIX: Publish done signal exactly once
        if self._done_published:
            return
        self._done_published = True
        msg = Bool()
        msg.data = success
        self.done_pub.publish(msg)
        self.get_logger().info(f"Entry done — success={success}")

    # ==================================================
    # Main Loop
    # ==================================================

    def loop(self):

        if self.STATE in ("STOP", "ABORTED"):
            return

        # --------------------------------------------------
        if self.STATE == "START_CURVE":
            self.send_cmd(0.2, -0.5)
            # FIX: ROS clock for duration check
            if self.elapsed(self.curve_start) > self.CURVE_DURATION:
                self.search_start = self.get_clock().now()
                self.transition("SEARCH_TAG")

        # --------------------------------------------------
        elif self.STATE == "SEARCH_TAG":
            # FIX: Timeout if tag never found
            if self.elapsed(self.search_start) > self.SEARCH_TIMEOUT:
                self.abort("Tag not found within timeout")
                return

            if not self.tag_is_visible():
                self.send_cmd(0.0, 0.4)
            else:
                self.transition("ALIGN_TAG")

        # --------------------------------------------------
        elif self.STATE == "ALIGN_TAG":
            # FIX: Abort if tag lost during alignment
            if not self.tag_is_visible():
                self.abort("Tag lost during alignment")
                return

            error = self.center_x - self.TARGET_X
            if abs(error) < self.ALIGN_TOLERANCE:
                self.transition("APPROACH_TAG")
            elif error > 0:
                self.send_cmd(0.0, -0.4)
            else:
                self.send_cmd(0.0,  0.4)

        # --------------------------------------------------
        elif self.STATE == "APPROACH_TAG":
            # FIX: Abort if tag lost during approach
            if not self.tag_is_visible():
                self.abort("Tag lost during approach")
                return

            error = self.center_x - self.TARGET_X

            # FIX: Transition to FINAL_FORWARD once well aligned
            if abs(error) < self.APPROACH_TOLERANCE:
                self.final_start = self.get_clock().now()   # FIX: set before entering state
                self.transition("FINAL_FORWARD")
                return

            if error > 0:
                self.send_cmd(0.25, -0.2)
            else:
                self.send_cmd(0.25,  0.2)

        # --------------------------------------------------
        elif self.STATE == "FINAL_FORWARD":
            # FIX: final_start is always set before this state is entered
            if self.elapsed(self.final_start) > self.FINAL_FORWARD_TIME:
                self.stop()
                self.transition("STOP")
                self._publish_done(success=True)   # FIX: published exactly once
            else:
                self.send_cmd(0.25, 0.0)


# ======================================================
# Main
# ======================================================

def main():
    rclpy.init()
    node = EntryNavigation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()