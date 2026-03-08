import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2


class CabbageDetector(Node):

    def __init__(self):
        super().__init__('cabbage_detector')

        self.model = YOLO('/home/jorjeen/plant/cabbage_project/cab_model/best.pt')

        self.bridge = CvBridge()

        self.CAMERA_HEIGHT = 32.5
        self.FOCAL_LENGTH = 440
        self.harvest_size_cm = 10.0

        self.last_cmd = ""

        self.subscription = self.create_subscription(
            Image,
            '/camera1/image_raw',
            self.image_callback,
            1
        )

        self.pub = self.create_publisher(
            Image,
            '/quin/image_detected',
            10
        )

        self.get_logger().info('Cabbage Detector started')

    # ==================================================

    def send_command(self, cmd):

        if cmd != self.last_cmd:
            self.get_logger().info(f"CMD: {cmd}")
            self.last_cmd = cmd

    # ==================================================

    def estimate_size(self, pixel_width):

        return (pixel_width * self.CAMERA_HEIGHT) / self.FOCAL_LENGTH

    # ==================================================

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        results = self.model(
            frame,
            imgsz=416,
            conf=0.5,
            verbose=False
        )[0]

        h, w = frame.shape[:2]

        top_bound = h * 0.15
        bottom_bound = h * 0.5

        cv2.line(frame, (0, int(top_bound)), (w, int(top_bound)), (255, 0, 0), 1)
        cv2.line(frame, (0, int(bottom_bound)), (w, int(bottom_bound)), (255, 0, 0), 1)

        if results.boxes is None or len(results.boxes) == 0:

            self.send_command("MOVE_FORWARD")

            cv2.putText(frame, "MOVE FORWARD", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255,255,0), 2)

        else:

            best_box = max(
                results.boxes,
                key=lambda b: (b.xyxy[0][2]-b.xyxy[0][0]) *
                              (b.xyxy[0][3]-b.xyxy[0][1])
            )

            x1, y1, x2, y2 = map(int, best_box.xyxy[0])

            center_y = (y1 + y2) / 2
            pixel_width = x2 - x1

            size_cm = self.estimate_size(pixel_width)

            self.get_logger().info(f"Cabbage size {size_cm:.1f} cm")

            cv2.rectangle(frame, (x1,y1),(x2,y2),(0,255,0),2)

            if top_bound <= center_y <= bottom_bound:

                self.send_command("STOP")

                if size_cm >= self.harvest_size_cm:

                    cv2.putText(frame,"READY TO HARVEST",(10,30),
                                cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255),2)

                else:

                    cv2.putText(frame,"NOT READY",(10,30),
                                cv2.FONT_HERSHEY_SIMPLEX,1,(0,165,255),2)

            else:

                self.send_command("MOVE_FORWARD")

        out_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.pub.publish(out_msg)


def main(args=None):

    rclpy.init(args=args)

    node = CabbageDetector()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()