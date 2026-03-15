import cv2
import threading
import time
import os

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Int32MultiArray, Bool, String

from ultralytics import YOLO
from pupil_apriltags import Detector


# =========================
# STREAM FROM PI
# =========================

PI_IP = "10.129.196.237"
STREAM1 = f"http://{PI_IP}:5000/stream1"
STREAM2 = f"http://{PI_IP}:5000/stream2"

os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "timeout;3000000"


# =========================
# LOAD MODELS
# =========================

enter_model  = YOLO('/home/jorjeen/plant/cabbage_project/entering_model/enterbest.pt')
cabbage_model = YOLO('/home/jorjeen/plant/cabbage_project/cab_model/best.pt')


# =========================
# APRILTAG DETECTOR
# =========================

at_detector = Detector(
    families="tagStandard52h13",
    nthreads=4,
    quad_decimate=2.0,
    quad_sigma=0.8,
    refine_edges=1,
    decode_sharpening=0.25
)


# =========================
# ROS2 NODE — MISSION PUBLISHER (AprilTag)
# =========================

class MissionPublisher(Node):
    def __init__(self):
        super().__init__('pc_cam_mission')
        self.pub = self.create_publisher(Int32MultiArray, '/mission_params', 10)
        self.last_tag_id = None

    def publish_mission(self, tag_id):
        if tag_id == self.last_tag_id:
            return

        decoded = self.decode_tag(tag_id)
        if decoded is None:
            self.get_logger().warn(f"Tag ID {tag_id} invalid C unit, skipping")
            return

        AB, C, DE = decoded
        self.last_tag_id = tag_id

        msg = Int32MultiArray()
        msg.data = [AB, C, DE]
        self.pub.publish(msg)
        self.get_logger().info(f"Mission published: AB={AB}cm  C={C}cm  DE={DE}cm  (tag_id={tag_id})")

    def decode_tag(self, tag_id):
        AB     = tag_id // 1000
        C_unit = (tag_id // 100) % 10
        DE     = tag_id % 100

        spacing_map = {1: 5, 2: 10, 3: 15, 4: 20, 5: 25}
        if C_unit not in spacing_map:
            return None

        C = spacing_map[C_unit]
        return AB, C, DE


# =========================
# ROS2 NODE — CABBAGE PUBLISHER
# =========================

class CabbagePublisher(Node):
    def __init__(self):
        super().__init__('pc_cam_cabbage')

        self.result_pub = self.create_publisher(String, '/quin/detect_result', 10)

        self._triggered = False
        self._trigger_lock = threading.Lock()

        self.create_subscription(Bool, '/quin/detect_trigger', self._trigger_callback, 10)

        # camera calibration — เดียวกับ cabbage_detector node
        self.CAMERA_HEIGHT = 32.5
        self.FOCAL_LENGTH  = 440

    def _trigger_callback(self, msg):
        if msg.data:
            with self._trigger_lock:
                self._triggered = True
            self.get_logger().info("Cabbage detection triggered")

    def is_triggered(self):
        with self._trigger_lock:
            return self._triggered

    def clear_trigger(self):
        with self._trigger_lock:
            self._triggered = False

    def estimate_size(self, pixel_width):
        return (pixel_width * self.CAMERA_HEIGHT) / self.FOCAL_LENGTH

    def publish_result(self, size_cm):
        msg = String()
        if size_cm > 0:
            msg.data = f"{size_cm:.1f},1"
        else:
            msg.data = "0,-1"
        self.result_pub.publish(msg)
        self.get_logger().info(f"Cabbage result published: {msg.data}")


# =========================
# FRESH FRAME READER
# =========================

class FreshFrame:
    def __init__(self, url):
        self.cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.lock = threading.Lock()
        self.latest_frame = None
        self.has_frame = False
        self.running = True

        threading.Thread(target=self.update, daemon=True).start()

    def update(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                with self.lock:
                    self.latest_frame = frame
                    self.has_frame = True
            time.sleep(0.001)

    def read(self):
        with self.lock:
            if not self.has_frame:
                return False, None
            return True, self.latest_frame.copy()


# =========================
# THREAD 1 — APRILTAG + ENTERING
# =========================

class EnteringThread:
    def __init__(self, stream, ros_node: MissionPublisher):
        self.stream   = stream
        self.ros_node = ros_node
        self.frame    = None
        self.running  = True
        self.last_infer = 0

        threading.Thread(target=self.run, daemon=True).start()

    def run(self):
        while self.running:
            ret, frame = self.stream.read()
            if not ret:
                continue

            draw_frame = frame.copy()
            h, w = draw_frame.shape[:2]
            target_x = int(w * 0.75)
            tag_center = None

            # ------------ APRILTAG ------------
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            gray = clahe.apply(gray)

            tags = at_detector.detect(gray)

            for tag in tags:
                if tag.decision_margin < 10.0:
                    continue

                corners = tag.corners.astype(int)
                for i in range(4):
                    cv2.line(draw_frame,
                             tuple(corners[i]),
                             tuple(corners[(i + 1) % 4]),
                             (0, 255, 0), 2)

                cx, cy = map(int, tag.center)
                tag_center = cx

                cv2.circle(draw_frame, (cx, cy), 5, (0, 0, 255), -1)
                cv2.putText(draw_frame,
                            f"ID:{tag.tag_id}",
                            (cx + 8, cy - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                self.ros_node.publish_mission(tag.tag_id)

            # ------------ YOLO ------------
            if time.time() - self.last_infer > 0.08:
                results = enter_model(
                    frame, imgsz=320, conf=0.25, device="cpu", verbose=False
                )[0]

                boxes = results.boxes
                if boxes is not None:
                    for box in boxes.xyxy:
                        x1, y1, x2, y2 = map(int, box)
                        cv2.rectangle(draw_frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                        cv2.putText(draw_frame, "ENTER", (x1, y1 - 5),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

                self.last_infer = time.time()

            # ------------ GRAPHICS ------------
            cv2.putText(draw_frame, "STATE: SEARCH TAG", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.putText(draw_frame, "ACTION: TRACK", (20, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

            cv2.line(draw_frame, (target_x, 0), (target_x, h), (255, 0, 0), 2)

            if tag_center is not None:
                cv2.line(draw_frame, (tag_center, 0), (tag_center, h), (0, 0, 255), 2)
                offset = tag_center - target_x
                cv2.putText(draw_frame, f"OFFSET: {offset}px", (20, 110),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 0), 2)

            self.frame = draw_frame


# =========================
# THREAD 2 — CABBAGE DETECTION
# =========================

class CabbageThread:
    def __init__(self, stream, ros_node: CabbagePublisher):
        self.stream   = stream
        self.ros_node = ros_node
        self.frame    = None
        self.running  = True
        self.last_infer = 0

        threading.Thread(target=self.run, daemon=True).start()

    def run(self):
        while self.running:
            ret, frame = self.stream.read()
            if not ret:
                continue

            draw_frame = frame.copy()

            if time.time() - self.last_infer > 0.08:

                results = cabbage_model(
                    frame, imgsz=320, conf=0.25, device="cpu", verbose=False
                )[0]

                boxes   = results.boxes
                size_cm = 0.0

                if boxes is not None and len(boxes) > 0:

                    # เอา box ที่ confidence สูงสุด
                    best_idx = int(boxes.conf.argmax())
                    box = boxes[best_idx]
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    pixel_width = x2 - x1
                    size_cm = self.ros_node.estimate_size(pixel_width)

                    # วาดทุก box
                    for b in boxes.xyxy:
                        bx1, by1, bx2, by2 = map(int, b)
                        cv2.rectangle(draw_frame, (bx1, by1), (bx2, by2), (0, 255, 0), 2)
                        cv2.putText(draw_frame, "CABBAGE", (bx1, by1 - 5),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                    # แสดงขนาดที่ best box
                    cv2.putText(draw_frame,
                                f"{size_cm:.1f} cm",
                                (x1, y2 + 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

                else:
                    size_cm = 0.0

                # ถ้า triggered → publish result แล้ว reset
                if self.ros_node.is_triggered():
                    self.ros_node.publish_result(size_cm)
                    self.ros_node.clear_trigger()

                self.last_infer = time.time()

            # แสดงสถานะ trigger บนหน้าจอ
            trigger_text = "TRIGGERED" if self.ros_node.is_triggered() else "WAITING"
            color = (0, 0, 255) if self.ros_node.is_triggered() else (100, 100, 100)
            cv2.putText(draw_frame, f"DETECT: {trigger_text}", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

            self.frame = draw_frame


# =========================
# MAIN
# =========================

def main():
    rclpy.init()

    mission_node = MissionPublisher()
    cabbage_node = CabbagePublisher()

    # รัน ROS2 ทั้ง 2 node ใน executor เดียว
    executor = MultiThreadedExecutor()
    executor.add_node(mission_node)
    executor.add_node(cabbage_node)

    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    print("Connecting streams...")
    stream1 = FreshFrame(STREAM1)
    stream2 = FreshFrame(STREAM2)
    time.sleep(2)

    print("Starting AI threads...")
    entering_thread = EnteringThread(stream1, mission_node)
    cabbage_thread  = CabbageThread(stream2, cabbage_node)

    fps_time    = time.time()
    fps_counter = 0

    while True:
        if entering_thread.frame is not None:
            display1 = cv2.resize(entering_thread.frame, (400, 300))
            cv2.imshow("Camera1 - AprilTag + Enter", display1)

        if cabbage_thread.frame is not None:
            display2 = cv2.resize(cabbage_thread.frame, (400, 300))
            cv2.imshow("Camera2 - Cabbage", display2)

        fps_counter += 1
        if time.time() - fps_time >= 1:
            print("FPS:", fps_counter)
            fps_counter = 0
            fps_time = time.time()

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
    mission_node.destroy_node()
    cabbage_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()