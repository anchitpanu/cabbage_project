# quin T1 :
# ros2 run usb_cam usb_cam_node_exe --ros-args \
# -p video_device:=/dev/video2 \
# -r __ns:=/camera2 \
# -p image_width:=320 \
# -p image_height:=240

# quin T2 : ros2 run web_video_server web_video_server
# web : http://10.129.196.237:8080/stream?topic=/camera2/image_detected


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2


class CabbageDetector(Node):

    def __init__(self):
        super().__init__('cabbage_detector')

        self.model  = YOLO('/home/jorjeen/plant/cabbage_project/cab_model/best.pt')
        self.bridge = CvBridge()

        self.CAMERA_HEIGHT    = 32.5
        self.FOCAL_LENGTH     = 440
        self.harvest_size_cm  = 10.0

        self.last_cmd    = ""
        self.frame_width  = 640
        self.frame_height = 480

        # -------- Trigger flag --------
        # True only when mission_node requests a single detection scan
        self._triggered = False

        # -------- Subscribers --------
        self.create_subscription(
            Image,
            '/camera2/image_raw',
            self.image_callback,
            1)

        # mission_node sends True here to request one detection scan
        self.create_subscription(
            Bool,
            '/quin/detect_trigger',
            self.detect_trigger_callback,
            10)
    
        # -------- Publishers --------
        self.pub = self.create_publisher(Image, '/camera2/image_detected', 10)

        # Sends result back to mission_node
        # Format: "size_cm,harvestable"
        #   e.g.  "12.4,0"  → not harvestable
        #         "18.7,1"  → harvestable
        #         "0.0,-1"  → not detected
        self.detect_result_pub = self.create_publisher(String, '/quin/detect_result', 10)

        self.get_logger().info('Cabbage Detector started — waiting for /quin/detect_trigger...')

    # ==================================================
    # Trigger from mission_node
    # ==================================================

    def detect_trigger_callback(self, msg: Bool):
        if not msg.data:
            return

        if self._triggered:
            self.get_logger().warn("Already scanning — duplicate trigger ignored")
            return

        self._triggered = True
        self.get_logger().info("Detection triggered — scanning next frame...")

    # ==================================================
    # Helpers
    # ==================================================

    def send_command(self, cmd):
        if cmd != self.last_cmd:
            print(f"CMD: {cmd}")
            self.last_cmd = cmd

    def estimate_size(self, pixel_width):
        return (pixel_width * self.CAMERA_HEIGHT) / self.FOCAL_LENGTH

    # ==================================================
    # Image Callback
    # ==================================================

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.frame_height, self.frame_width = frame.shape[:2]

        # Resize for YOLO
        frame_small = cv2.resize(frame, (320, 320))
        scale_x = frame.shape[1] / 320
        scale_y = frame.shape[0] / 320

        results = self.model(
            frame_small,
            conf=0.5,
            imgsz=320,
            device="cpu",
            verbose=False
        )[0]

        top_bound    = self.frame_height * 0.15
        bottom_bound = self.frame_height * 0.5

        cv2.line(frame, (0, int(top_bound)),    (self.frame_width, int(top_bound)),    (255, 0, 0), 1)
        cv2.line(frame, (0, int(bottom_bound)), (self.frame_width, int(bottom_bound)), (255, 0, 0), 1)

        if len(results.boxes) == 0:

            self.send_command("MOVE_FORWARD")
            cv2.putText(frame, "MOVE FORWARD", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), 2)

            # If triggered but nothing detected → report not detected
            if self._triggered:
                self._publish_result(size_cm=0.0, harvestable=-1)
                self._triggered = False

        else:
            best_box = max(results.boxes,
                           key=lambda b: (b.xyxy[0][2] - b.xyxy[0][0]) *
                                         (b.xyxy[0][3] - b.xyxy[0][1]))

            x1, y1, x2, y2 = best_box.xyxy[0]

            x1 = int(x1 * scale_x)
            x2 = int(x2 * scale_x)
            y1 = int(y1 * scale_y)
            y2 = int(y2 * scale_y)

            center_y    = (y1 + y2) / 2
            pixel_width = x2 - x1
            size_cm     = self.estimate_size(pixel_width)

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"{size_cm:.1f} cm", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            if top_bound <= center_y <= bottom_bound:

                self.send_command("STOP")
                self.get_logger().info(f"Cabbage detected — size: {size_cm:.1f} cm")

                if size_cm >= self.harvest_size_cm:
                    harvestable = 1
                    self.get_logger().info("Status: READY TO HARVEST")
                    cv2.putText(frame, "READY TO HARVEST", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
                else:
                    harvestable = 0
                    self.get_logger().info("Status: NOT READY")
                    cv2.putText(frame, "NOT READY", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 165, 255), 2)

                # Publish result back to mission_node if this was a triggered scan
                if self._triggered:
                    self._publish_result(size_cm=size_cm, harvestable=harvestable)
                    self._triggered = False

            else:
                self.send_command("MOVE_FORWARD")
                cv2.putText(frame, "MOVE FORWARD", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), 2)

                # Cabbage visible but not in detection zone yet — keep waiting
                # (mission_node detect timeout will handle if it never enters zone)

        out_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.pub.publish(out_msg)

    # ==================================================
    # Publish Result to mission_node
    # ==================================================

    def _publish_result(self, size_cm: float, harvestable: int):
        """
        Sends detection result to mission_node.
        Format: "size_cm,harvestable"
          harvestable:  1 = ready,  0 = not ready,  -1 = not detected
        """
        msg      = String()
        msg.data = f"{size_cm:.1f},{harvestable}"
        self.detect_result_pub.publish(msg)
        self.get_logger().info(f"Result published → {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = CabbageDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()