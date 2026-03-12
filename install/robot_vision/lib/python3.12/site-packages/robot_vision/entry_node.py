# quin T1 :
# ros2 run usb_cam usb_cam_node_exe --ros-args \
# -p video_device:=/dev/video0 \
# -r __ns:=/camera1 \
# -p image_width:=320 \
# -p image_height:=240

# quin T2 :
# ros2 run web_video_server web_video_server

# web :
# http://10.129.196.237:8080/stream?topic=/camera1/image_detected


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import time


class EnterBox(Node):

    def __init__(self):

        super().__init__('enter_box_node')

        self.model = YOLO('/home/jorjeen/plant/cabbage_project/entering_model/enterbest.pt')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera1/image_raw',
            self.image_callback,
            1)

        self.pub = self.create_publisher(Image, '/camera1/image_detected', 10)

        # Listens for mission_node trigger to start entry
        self.create_subscription(
            Bool,
            '/entry_start',
            self.entry_start_callback,
            10)

        # Publishes True to mission_node when robot has fully entered the box
        self.entry_done_pub = self.create_publisher(Bool, '/entry_done', 10)

        # -------- STATE MACHINE --------

        self.STATE = "WAITING"          # wait for /entry_start before doing anything

        self.curve_start     = None
        self.final_start     = None

        self.TARGET_X            = 160
        self.FINAL_FORWARD_TIME  = 1.6

        self.lost_counter = 0
        self.tag_seen     = False

        self.prev_center = None
        self.last_center = None

        self.last_cmd = ""

        # Prevents publishing /entry_done more than once per run
        self._entry_done_sent = False

        # frame skip
        self.frame_count = 0

        self.get_logger().info("Enter Box Node Started — waiting for /entry_start...")

    # ==================================================
    # Entry Start Trigger from mission_node
    # ==================================================

    def entry_start_callback(self, msg: Bool):
        if not msg.data:
            return

        if self.STATE != "WAITING":
            self.get_logger().warn("Entry already in progress — /entry_start ignored")
            return

        self.get_logger().info("Entry start received — beginning entry sequence")

        # Reset all state for a clean run
        self.STATE            = "START_CURVE"
        self.curve_start      = time.time()
        self.final_start      = None
        self.lost_counter     = 0
        self.tag_seen         = False
        self.prev_center      = None
        self.last_center      = None
        self.last_cmd         = ""
        self._entry_done_sent = False
        self.frame_count      = 0

    # ==================================================
    # Command Helper
    # ==================================================

    def send_command(self, cmd):

        if cmd != self.last_cmd:
            print(f"CMD: {cmd}")
            self.last_cmd = cmd

    # ==================================================
    # Image Callback — YOLO + State Machine
    # ==================================================

    def image_callback(self, msg):

        # -------- FRAME SKIP --------
        self.frame_count += 1
        if self.frame_count % 2 != 0:
            return

        # Do nothing until entry_start is received
        if self.STATE == "WAITING":
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        h, w = frame.shape[:2]

        # -------- YOLO INPUT SMALL --------
        frame_small = cv2.resize(frame, (256, 256))

        scale_x = w / 256
        scale_y = h / 256

        results = self.model(
            frame_small,
            conf=0.35,
            imgsz=256,
            device="cpu",
            verbose=False
        )[0]

        action_text = "NONE"

        tag_detected = False
        center_x     = None

        boxes = results.boxes

        # -------- SELECT SMALLEST TAG --------

        if boxes is not None and len(boxes) > 0:

            smallest_area = 999999999
            best_box      = None

            for box in boxes.xyxy:

                x1, y1, x2, y2 = box
                area = float((x2 - x1) * (y2 - y1))

                if area < 60000 and area < smallest_area:
                    smallest_area = area
                    best_box      = box

            if best_box is not None:

                x1, y1, x2, y2 = best_box

                x1 = int(x1 * scale_x)
                x2 = int(x2 * scale_x)
                y1 = int(y1 * scale_y)
                y2 = int(y2 * scale_y)

                center_x = float((x1 + x2) / 2)

                tag_detected      = True
                self.tag_seen     = True
                self.lost_counter = 0

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

            else:
                self.lost_counter += 1

        else:
            self.lost_counter += 1

        # -------- HOLD DETECTION --------

        if tag_detected:
            self.last_center = center_x
        elif self.last_center is not None:
            center_x = self.last_center

        # -------- SMOOTH CENTER --------

        if center_x is not None and self.prev_center is not None:
            center_x = 0.7 * self.prev_center + 0.3 * center_x

        self.prev_center = center_x

        # -------- STATE MACHINE --------

        if self.STATE == "START_CURVE":

            action_text = "CURVE RIGHT"
            self.send_command(action_text)

            if time.time() - self.curve_start > 2.5:
                self.STATE = "SEARCH_TAG"

        elif self.STATE == "SEARCH_TAG":

            if not tag_detected:
                action_text = "ROTATE LEFT"
                self.send_command(action_text)
            else:
                self.STATE = "ALIGN_TAG"

        elif self.STATE == "ALIGN_TAG":

            if center_x is None:
                self.STATE = "SEARCH_TAG"

            else:
                error = center_x - self.TARGET_X

                if abs(error) < 5:
                    self.STATE = "APPROACH_TAG"

                elif error > 0:
                    action_text = "TURN RIGHT"
                    self.send_command(action_text)

                else:
                    action_text = "TURN LEFT"
                    self.send_command(action_text)

        elif self.STATE == "APPROACH_TAG":

            if tag_detected:

                error = center_x - self.TARGET_X

                if abs(error) > 20:
                    self.STATE = "ALIGN_TAG"

                else:
                    if abs(error) < 8:
                        action_text = "FORWARD"
                    elif error > 0:
                        action_text = "FORWARD_RIGHT"
                    else:
                        action_text = "FORWARD_LEFT"

                    self.send_command(action_text)

            else:
                if self.tag_seen and self.lost_counter > 5:
                    self.STATE       = "FINAL_FORWARD"
                    self.final_start = time.time()
                else:
                    action_text = "TAG LOST"

        elif self.STATE == "FINAL_FORWARD":

            action_text = "FORWARD"
            self.send_command(action_text)

            if time.time() - self.final_start > self.FINAL_FORWARD_TIME:
                self.STATE = "STOP"

        elif self.STATE == "STOP":

            action_text = "STOP"
            self.send_command(action_text)

            # ---- Notify mission_node that entry is complete ----
            if not self._entry_done_sent:
                done_msg      = Bool()
                done_msg.data = True
                self.entry_done_pub.publish(done_msg)
                self._entry_done_sent = True
                self.get_logger().info("Entry complete — /entry_done published to mission_node")

        # -------- VISUAL DEBUG --------

        cv2.putText(frame, f"STATE: {self.STATE}",
                    (30, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 0),
                    2)

        cv2.putText(frame, f"ACTION: {action_text}",
                    (30, 70),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 255),
                    2)

        if center_x is not None:
            cv2.line(frame,
                     (int(center_x), 0),
                     (int(center_x), h),
                     (0, 0, 255),
                     2)

        cv2.line(frame,
                 (self.TARGET_X, 0),
                 (self.TARGET_X, h),
                 (255, 0, 0),
                 2)

        out_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.pub.publish(out_msg)


def main(args=None):

    rclpy.init(args=args)

    node = EnterBox()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()