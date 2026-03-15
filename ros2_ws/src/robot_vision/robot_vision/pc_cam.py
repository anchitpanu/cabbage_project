import cv2
import numpy as np
import threading
import time
import os

from ultralytics import YOLO
from pupil_apriltags import Detector

# =========================
# STREAM FROM PI
# =========================

PI_IP = "10.129.196.237"

STREAM1 = f"http://10.129.196.237:5000/stream1"
STREAM2 = f"http://10.129.196.237:5000/stream2"

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
    quad_decimate=2.0,
)

# =========================
# FRESH FRAME CLASS
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

        threading.Thread(target=self.run, daemon=True).start()

    def run(self):

        while self.running:

            ret, frame = self.stream.read()

            if not ret:
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            tags = at_detector.detect(gray)

            for tag in tags:

                corners = tag.corners.astype(int)

                for i in range(4):

                    cv2.line(frame,
                             tuple(corners[i]),
                             tuple(corners[(i+1)%4]),
                             (0,255,0),2)

            # YOLO entering
            results = enter_model(
                frame,
                imgsz=320,
                conf=0.25,
                stream=True,
                verbose=False
    )

            for r in results:

                boxes = r.boxes.xyxy.cpu().numpy()

                for box in boxes:

                    x1,y1,x2,y2 = map(int,box)

                    cv2.rectangle(frame,(x1,y1),(x2,y2),(255,0,0),2)

                    cv2.putText(frame,"ENTER",
                                (x1,y1-5),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.6,
                                (255,0,0),
                                2)

            self.frame = frame

# =========================
# THREAD 2
# CABBAGE DETECTION
# =========================

class CabbageThread:

    def __init__(self, stream):

        self.stream = stream
        self.frame = None

        self.running = True

        threading.Thread(target=self.run, daemon=True).start()

    def run(self):

        while self.running:

            ret, frame = self.stream.read()

            if not ret:
                continue

            results = cabbage_model(
                frame,
                imgsz=320,
                conf=0.25,
                stream=True,
                verbose=False
    )

            for r in results:

                boxes = r.boxes.xyxy.cpu().numpy()

                for box in boxes:

                    x1,y1,x2,y2 = map(int,box)

                    cv2.rectangle(frame,(x1,y1),(x2,y2),(0,255,0),2)

                    cv2.putText(frame,"CABBAGE",
                                (x1,y1-5),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.6,
                                (0,255,0),
                                2)

            self.frame = frame

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

    while True:

        if entering_thread.frame is not None:

            display1 = cv2.resize(entering_thread.frame, (400,300))
            cv2.imshow("Camera1 - AprilTag + Enter", display1)

        if cabbage_thread.frame is not None:

            display2 = cv2.resize(cabbage_thread.frame, (400,300))
            cv2.imshow("Camera2 - Cabbage", display2)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()