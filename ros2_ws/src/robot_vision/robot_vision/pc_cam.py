import cv2
import threading
import time
import os

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

enter_model = YOLO('/home/jorjeen/plant/cabbage_project/entering_model/enterbest.pt')
cabbage_model = YOLO('/home/jorjeen/plant/cabbage_project/cab_model/best.pt')


# =========================
# APRILTAG DETECTOR
# =========================

at_detector = Detector(
    families="tagStandard52h13",
    nthreads=4,
    quad_decimate=2.0
)


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
# THREAD 1
# APRILTAG + ENTERING
# =========================

class EnteringThread:

    def __init__(self, stream):

        self.stream = stream
        self.frame = None

        self.running = True
        self.last_infer = 0

        threading.Thread(target=self.run, daemon=True).start()

    def run(self):

        while self.running:

            ret, frame = self.stream.read()

            if not ret:
                continue

            draw_frame = frame.copy()

            # -------- AprilTag --------

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            tags = at_detector.detect(gray)

            for tag in tags:

                corners = tag.corners.astype(int)

                for i in range(4):
                    cv2.line(draw_frame,
                             tuple(corners[i]),
                             tuple(corners[(i+1)%4]),
                             (0,255,0),2)

            # -------- YOLO --------

            if time.time() - self.last_infer > 0.08:

                results = enter_model(
                    frame,
                    imgsz=320,
                    conf=0.25,
                    device="cpu",
                    verbose=False
                )[0]

                boxes = results.boxes

                if boxes is not None:

                    for box in boxes.xyxy:

                        x1,y1,x2,y2 = map(int,box)

                        cv2.rectangle(draw_frame,(x1,y1),(x2,y2),(255,0,0),2)

                        cv2.putText(draw_frame,"ENTER",
                                    (x1,y1-5),
                                    cv2.FONT_HERSHEY_SIMPLEX,
                                    0.6,
                                    (255,0,0),2)

                self.last_infer = time.time()

            self.frame = draw_frame


# =========================
# THREAD 2
# CABBAGE DETECTION
# =========================

class CabbageThread:

    def __init__(self, stream):

        self.stream = stream
        self.frame = None

        self.running = True
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
                    frame,
                    imgsz=320,
                    conf=0.25,
                    device="cpu",
                    verbose=False
                )[0]

                boxes = results.boxes

                if boxes is not None:

                    for box in boxes.xyxy:

                        x1,y1,x2,y2 = map(int,box)

                        cv2.rectangle(draw_frame,(x1,y1),(x2,y2),(0,255,0),2)

                        cv2.putText(draw_frame,"CABBAGE",
                                    (x1,y1-5),
                                    cv2.FONT_HERSHEY_SIMPLEX,
                                    0.6,
                                    (0,255,0),2)

                self.last_infer = time.time()

            self.frame = draw_frame


# =========================
# MAIN
# =========================

def main():

    print("Connecting streams...")

    stream1 = FreshFrame(STREAM1)
    stream2 = FreshFrame(STREAM2)

    time.sleep(2)

    print("Starting AI threads...")

    entering_thread = EnteringThread(stream1)
    cabbage_thread = CabbageThread(stream2)

    fps_time = time.time()
    fps_counter = 0

    while True:

        if entering_thread.frame is not None:

            display1 = cv2.resize(entering_thread.frame,(400,300))
            cv2.imshow("Camera1 - AprilTag + Enter",display1)

        if cabbage_thread.frame is not None:

            display2 = cv2.resize(cabbage_thread.frame,(400,300))
            cv2.imshow("Camera2 - Cabbage",display2)

        fps_counter += 1

        if time.time() - fps_time >= 1:

            print("FPS:",fps_counter)

            fps_counter = 0
            fps_time = time.time()

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()