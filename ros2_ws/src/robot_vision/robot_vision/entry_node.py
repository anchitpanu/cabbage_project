import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import time


class EnterBox(Node):

    def __init__(self):

        super().__init__('enter_box_node')

        self.model = YOLO('/home/jorjeen/plant/cabbage_project/entering_model/enterbest.pt')

        self.bridge = CvBridge()

        self.create_subscription(
            Image,
            '/camera1/image_raw',
            self.image_callback,
            1)

        self.pub = self.create_publisher(
            Image,
            '/camera1/image_detected',
            10)

        self.cmd_pub = self.create_publisher(
            Twist,
            '/quin/move_command',
            10)

        self.create_subscription(
            Bool,
            '/entry_start',
            self.entry_start_callback,
            10)

        self.entry_done_pub = self.create_publisher(
            Bool,
            '/entry_done',
            10)

        self.STATE = "WAITING"

        self.TARGET_RATIO = 0.82
        self.FINAL_FORWARD_TIME = 1.3

        self.last_cmd = ""

        self.tag_seen = False
        self.lost_counter = 0

        self.prev_center = None
        self.last_center = None

        self.final_start = None

        self._entry_done_sent = False
        self._completed_final_forward = False

        self.last_pub_time = 0
        self.PUB_INTERVAL = 0.1

        self.last_infer = 0

        self.get_logger().info("Enter Box Node Ready")


    # -------------------------------------------------

    def entry_start_callback(self, msg):

        if not msg.data:
            return

        if self.STATE != "WAITING":
            return

        self.get_logger().info("Entry started")

        self.STATE = "SEARCH_TAG"

        self.tag_seen = False
        self.lost_counter = 0
        self.prev_center = None
        self.last_center = None

        self._entry_done_sent = False
        self._completed_final_forward = False


    # -------------------------------------------------

    def send_command(self, cmd):

        if cmd == self.last_cmd:
            return

        self.last_cmd = cmd

        msg = Twist()

        if cmd == "FORWARD":
            msg.linear.x = 0.2

        elif cmd == "FORWARD_LEFT":
            msg.linear.x = 0.15
            msg.angular.z = 0.3

        elif cmd == "FORWARD_RIGHT":
            msg.linear.x = 0.15
            msg.angular.z = -0.3

        elif cmd == "TURN_LEFT":
            msg.angular.z = 0.5

        elif cmd == "TURN_RIGHT":
            msg.angular.z = -0.5

        elif cmd == "ROTATE_LEFT":
            msg.angular.z = 0.6

        elif cmd == "STOP":
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        self.cmd_pub.publish(msg)


    # -------------------------------------------------

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        h, w = frame.shape[:2]

        target_x = int(w * self.TARGET_RATIO)

        center_x = None
        action_text = "NONE"

        # ---------------- YOLO (จำกัด FPS)

        if time.time() - self.last_infer > 0.07:

            frame_small = cv2.resize(frame, (256,256))

            scale_x = w / 256
            scale_y = h / 256

            results = self.model(
                frame_small,
                conf=0.25,
                imgsz=192,
                device="cpu",
                verbose=False
            )[0]

            boxes = results.boxes

            if boxes is not None and len(boxes) > 0:

                best_box = None
                smallest_area = 1e9

                for box in boxes.xyxy:

                    x1,y1,x2,y2 = box
                    area = float((x2-x1)*(y2-y1))

                    if area < smallest_area:
                        smallest_area = area
                        best_box = box

                if best_box is not None:

                    x1,y1,x2,y2 = best_box

                    x1 = int(x1 * scale_x)
                    x2 = int(x2 * scale_x)
                    y1 = int(y1 * scale_y)
                    y2 = int(y2 * scale_y)

                    center_x = int((x1+x2)/2)

                    self.tag_seen = True
                    self.lost_counter = 0

                    cv2.rectangle(frame,(x1,y1),(x2,y2),(0,255,0),2)

            else:

                self.lost_counter += 1

            self.last_infer = time.time()


        # ---------------- STATE MACHINE

        if self.STATE == "WAITING":

            action_text = "WAITING"

        elif self.STATE == "SEARCH_TAG":

            if center_x is not None:

                self.STATE = "ALIGN_TAG"

            else:

                action_text = "ROTATE_LEFT"
                self.send_command("ROTATE_LEFT")


        elif self.STATE == "ALIGN_TAG":

            if center_x is not None:

                error = center_x - target_x

                if abs(error) < 5:

                    self.send_command("STOP")
                    self.STATE = "APPROACH_TAG"

                elif error > 0:

                    action_text = "TURN_RIGHT"
                    self.send_command("TURN_RIGHT")

                else:

                    action_text = "TURN_LEFT"
                    self.send_command("TURN_LEFT")


        elif self.STATE == "APPROACH_TAG":

            if center_x is not None:

                error = center_x - target_x

                if abs(error) < 8:

                    action_text = "FORWARD"
                    self.send_command("FORWARD")

                elif error > 0:

                    action_text = "FORWARD_RIGHT"
                    self.send_command("FORWARD_RIGHT")

                else:

                    action_text = "FORWARD_LEFT"
                    self.send_command("FORWARD_LEFT")

            else:

                if self.tag_seen and self.lost_counter > 12:

                    self.STATE = "FINAL_FORWARD"
                    self.final_start = time.time()


        elif self.STATE == "FINAL_FORWARD":

            action_text = "FORWARD"
            self.send_command("FORWARD")

            if time.time() - self.final_start > self.FINAL_FORWARD_TIME:

                self._completed_final_forward = True
                self.STATE = "STOP"


        elif self.STATE == "STOP":

            self.send_command("STOP")

            if not self._entry_done_sent:

                msg_done = Bool()
                msg_done.data = self._completed_final_forward

                self.entry_done_pub.publish(msg_done)

                self._entry_done_sent = True


        # ---------------- DRAW GRAPHICS

        cv2.putText(frame,
                    f"STATE: {self.STATE}",
                    (20,40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0,255,0),
                    2)

        cv2.putText(frame,
                    f"ACTION: {action_text}",
                    (20,80),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0,255,255),
                    2)

        if center_x is not None:

            cv2.line(frame,
                     (center_x,0),
                     (center_x,h),
                     (0,0,255),
                     2)

        cv2.line(frame,
                 (target_x,0),
                 (target_x,h),
                 (255,0,0),
                 2)


        # ---------------- PUBLISH FRAME

        if time.time() - self.last_pub_time > self.PUB_INTERVAL:

            display = cv2.resize(frame,(320,240))

            out_msg = self.bridge.cv2_to_imgmsg(display,'bgr8')

            self.pub.publish(out_msg)

            self.last_pub_time = time.time()


def main():

    rclpy.init()

    node = EnterBox()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()