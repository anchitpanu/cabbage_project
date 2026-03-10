
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
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class CabbageDetector(Node):
    def __init__(self):
        super().__init__('cabbage_detector')
        self.model = YOLO('/home/quin/cabbage_project/cab_model/best.pt')
        self.bridge = CvBridge()
        
        self.CAMERA_HEIGHT = 32.5
        self.FOCAL_LENGTH = 440
        self.harvest_size_cm = 10.0
        
        self.last_cmd = ""
        self.frame_width = 640
        self.frame_height = 480
        self.subscription = self.create_subscription(
            Image, '/camera2/image_raw', self.image_callback, 1)
        self.pub = self.create_publisher(Image, '/camera2/image_detected', 10)
        self.get_logger().info('Cabbage Detector started!')

    def send_command(self, cmd):
        if cmd != self.last_cmd:
            print(f"CMD: {cmd}")
            self.last_cmd = cmd

    def estimate_size(self, pixel_width):
        return (pixel_width * self.CAMERA_HEIGHT) / self.FOCAL_LENGTH

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.frame_height, self.frame_width = frame.shape[:2]

        # ลดขนาดภาพก่อนเข้า YOLO
        frame_small = cv2.resize(frame, (320, 320))

        # scale สำหรับแปลง bounding box กลับ
        scale_x = frame.shape[1] / 320
        scale_y = frame.shape[0] / 320

        results = self.model(frame_small, conf=0.5, imgsz=320, device="cpu", verbose=False)[0]

        top_bound = self.frame_height * 0.15
        bottom_bound = self.frame_height * 0.5
        cv2.line(frame, (0, int(top_bound)), (self.frame_width, int(top_bound)), (255, 0, 0), 1)
        cv2.line(frame, (0, int(bottom_bound)), (self.frame_width, int(bottom_bound)), (255, 0, 0), 1)

        if len(results.boxes) == 0:
            self.send_command("MOVE_FORWARD")
            cv2.putText(frame, "MOVE FORWARD", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), 2)
        else:
            best_box = max(results.boxes,
                           key=lambda b: (b.xyxy[0][2]-b.xyxy[0][0]) *
                                         (b.xyxy[0][3]-b.xyxy[0][1]))
            x1, y1, x2, y2 = best_box.xyxy[0]

            x1 = int(x1 * scale_x)
            x2 = int(x2 * scale_x)
            y1 = int(y1 * scale_y)
            y2 = int(y2 * scale_y)

            center_y = (y1 + y2) / 2
            pixel_width = x2 - x1
            size_cm = self.estimate_size(pixel_width)

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"{size_cm:.1f} cm", (x1, y1-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            if top_bound <= center_y <= bottom_bound:
                self.send_command("STOP")
                print(f"พบบอล ขนาด: {size_cm:.1f} ซม.")
                if size_cm >= self.harvest_size_cm:
                    print("READY_TO_HARVEST")
                    cv2.putText(frame, "READY TO HARVEST", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
                else:
                    print("ยังไม่ถึงขนาด")
                    cv2.putText(frame, "NOT READY", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 165, 255), 2)
            else:
                self.send_command("MOVE_FORWARD")
                cv2.putText(frame, "MOVE FORWARD", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), 2)

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