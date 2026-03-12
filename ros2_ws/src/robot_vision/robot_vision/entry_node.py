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

        self.model = YOLO('/home/quin/cabbage_project/entering_model/enterbest.pt')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera1/image_raw',
            self.image_callback,
            1)

        self.pub = self.create_publisher(Image, '/camera1/image_detected', 10)

        self.cmd_pub = self.create_publisher(
            Twist,
            '/quin/move_command',
            10)

        self.create_subscription(
            Bool,
            '/entry_start',
            self.entry_start_callback,
            10)

        self.entry_done_pub = self.create_publisher(Bool, '/entry_done', 10)

        self.STATE = "WAITING"

        self.curve_start = None
        self.final_start = None

        self.TARGET_RATIO = 0.82
        self.FINAL_FORWARD_TIME = 1.3

        self.lost_counter = 0
        self.tag_seen = False

        self.prev_center = None
        self.last_center = None

        self.last_cmd = ""
        self._entry_done_sent = False

        self.frame_count = 0

        self.get_logger().info("Enter Box Node Started — waiting for /entry_start...")

    # -------------------------------------------------

    def entry_start_callback(self, msg: Bool):

        if not msg.data:
            return

        if self.STATE != "WAITING":
            return

        self.get_logger().info("Entry start received")

        self.STATE = "START_CURVE"

        self.curve_start = time.time()
        self.final_start = None

        self.lost_counter = 0
        self.tag_seen = False
        self.prev_center = None
        self.last_center = None

        self.last_cmd = ""
        self._entry_done_sent = False
        self.frame_count = 0

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

        elif cmd == "CURVE_RIGHT":
            msg.linear.x = 0.1
            msg.angular.z = -0.5

        elif cmd == "STOP":
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        self.cmd_pub.publish(msg)

    # -------------------------------------------------

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        h, w = frame.shape[:2]

        target_x = int(w * self.TARGET_RATIO)

        self.frame_count += 1
        if self.frame_count % 2 != 0:
            return

        frame_small = cv2.resize(frame, (256, 256))

        scale_x = w / 256
        scale_y = h / 256

        results = self.model(
            frame_small,
            conf=0.25,
            imgsz=256,
            device="cpu",
            verbose=False
        )[0]

        action_text = "NONE"

        tag_detected = False
        center_x = None

        boxes = results.boxes

        if boxes is not None and len(boxes) > 0:

            smallest_area = 999999999
            best_box = None

            for box in boxes.xyxy:

                x1, y1, x2, y2 = box
                area = float((x2 - x1) * (y2 - y1))

                if area < smallest_area:
                    smallest_area = area
                    best_box = box

            if best_box is not None:

                x1, y1, x2, y2 = best_box

                x1 = int(x1 * scale_x)
                x2 = int(x2 * scale_x)
                y1 = int(y1 * scale_y)
                y2 = int(y2 * scale_y)

                center_x = float((x1 + x2) / 2)

                tag_detected = True
                self.tag_seen = True
                self.lost_counter = 0

                cv2.rectangle(frame,(x1,y1),(x2,y2),(0,255,0),2)

            else:
                self.lost_counter += 1
        else:
            self.lost_counter += 1

        if tag_detected:
            self.last_center = center_x
        elif self.last_center is not None:
            center_x = self.last_center

        if center_x is not None and self.prev_center is not None:
            center_x = 0.7 * self.prev_center + 0.3 * center_x

        self.prev_center = center_x

        if self.STATE != "WAITING":

            if self.STATE == "START_CURVE":

                action_text = "CURVE RIGHT"
                self.send_command(action_text)

                if time.time() - self.curve_start > 2.5:
                    self.STATE = "SEARCH_TAG"

            elif self.STATE == "SEARCH_TAG":

                if not tag_detected:
                    action_text = "ROTATE_LEFT"
                    self.send_command(action_text)
                else:
                    self.STATE = "ALIGN_TAG"

            elif self.STATE == "ALIGN_TAG":

                if center_x is None:
                    self.STATE = "SEARCH_TAG"

                else:

                    error = center_x - target_x

                    if abs(error) < 5:
                        self.STATE = "APPROACH_TAG"

                    elif error > 0:
                        action_text = "TURN_RIGHT"
                        self.send_command(action_text)

                    else:
                        action_text = "TURN_LEFT"
                        self.send_command(action_text)

            elif self.STATE == "APPROACH_TAG":

                if tag_detected:

                    error = center_x - target_x

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
                        self.STATE = "FINAL_FORWARD"
                        self.final_start = time.time()

            elif self.STATE == "FINAL_FORWARD":

                action_text = "FORWARD"
                self.send_command(action_text)

                if time.time() - self.final_start > self.FINAL_FORWARD_TIME:
                    self.STATE = "STOP"

            elif self.STATE == "STOP":

                action_text = "STOP"
                self.send_command(action_text)

                if not self._entry_done_sent:

                    done_msg = Bool()
                    done_msg.data = True

                    self.entry_done_pub.publish(done_msg)

                    self._entry_done_sent = True

        cv2.putText(frame,f"STATE: {self.STATE}",(30,40),
                    cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,255,0),2)

        cv2.putText(frame,f"ACTION: {action_text}",(30,70),
                    cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,255,255),2)

        if center_x is not None:
            cv2.line(frame,(int(center_x),0),(int(center_x),h),(0,0,255),2)

        cv2.line(frame,(target_x,0),(target_x,h),(255,0,0),2)

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