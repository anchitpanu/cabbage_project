import cv2
from flask import Flask, Response
import threading
import time
import queue

app = Flask(__name__)

WIDTH = 480
HEIGHT = 360
FPS = 10
JPEG_QUALITY = 18

# =====================
# CAMERA SERVER CLASS
# =====================

class CameraServer:

    def __init__(self, index):

        self.index = index

        self.cap = cv2.VideoCapture(index, cv2.CAP_V4L2)

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, FPS)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

        self.lock = threading.Lock()
        self.frame = None

        self.running = True

        threading.Thread(target=self.update, daemon=True).start()

    def update(self):

        while self.running:

            ret, frame = self.cap.read()

            if ret:

                with self.lock:
                    self.frame = frame

    def get(self):

        with self.lock:

            if self.frame is None:
                return False, None

            return True, self.frame.copy()

# =====================
# CREATE CAMERAS
# =====================

cam1 = CameraServer(0)
cam2 = CameraServer(2)

# =====================
# STREAM GENERATOR
# =====================

def generate(cam):

    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]

    while True:

        ret, frame = cam.get()

        if not ret:
            time.sleep(0.01)
            continue

        ret, buf = cv2.imencode('.jpg', frame, encode_param)

        frame = buf.tobytes()

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

# =====================
# ROUTES
# =====================

@app.route('/stream1')
def stream1():
    return Response(generate(cam1),
        mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/stream2')
def stream2():
    return Response(generate(cam2),
        mimetype='multipart/x-mixed-replace; boundary=frame')

# =====================
# MAIN
# =====================

if __name__ == "__main__":

    print("Camera server started")

    app.run(
        host="0.0.0.0",
        port=5000,
        threaded=True,
        debug=False
    )