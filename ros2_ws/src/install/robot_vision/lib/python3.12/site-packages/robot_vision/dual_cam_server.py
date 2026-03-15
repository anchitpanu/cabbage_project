#!/usr/bin/env python3
"""
dual_cam_server.py  — รันบน Pi 5
วางไว้ที่: ~/dual_cam_server.py

Stream กล้อง 2 ตัวพร้อมกัน:
  port 5000 → CAMERA_INDEX_1  (AprilTag / Enter)
  port 5001 → CAMERA_INDEX_2  (Cabbage)

Usage:
  python3 dual_cam_server.py

เช็ค index กล้องก่อน:
  v4l2-ctl --list-devices
  ls /dev/video*
"""

import cv2
import threading
import time
import queue
from flask import Flask, Response

# ── Config เลข usb cam ───────────────────────────────────────────────────────────
CAMERA_INDEX_1 = 2      # กล้อง AprilTag/Enter
CAMERA_INDEX_2 = 0      # กล้อง Cabbage 

WIDTH, HEIGHT  = 640, 480
FPS            = 15
JPEG_QUALITY   = 40
# ─────────────────────────────────────────────────────────────────────


class CameraServer:

    def __init__(self, index, name=""):
        self.index     = index
        self.name      = name
        self.running   = True
        self.lock      = threading.Lock()
        self.latest_frame = None
        self.has_frame    = False
        threading.Thread(target=self._capture_loop, daemon=True).start()

    def _capture_loop(self):
        while self.running:
            cap = cv2.VideoCapture(self.index, cv2.CAP_V4L2)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH,  WIDTH)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
            cap.set(cv2.CAP_PROP_FPS,          FPS)
            cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

            if not cap.isOpened():
                print(f"[{self.name}] ไม่เจอกล้อง index={self.index} — รอ 3s...")
                time.sleep(3)
                continue

            print(f"[{self.name}] warm-up...")
            for _ in range(8):
                cap.read()
            print(f"[{self.name}] พร้อมแล้ว ✓")

            while self.running:
                ret, frame = cap.read()
                if not ret or frame is None:
                    print(f"[{self.name}] กล้องหลุด — reconnect")
                    break
                with self.lock:
                    self.latest_frame = frame
                    self.has_frame    = True

            cap.release()
            time.sleep(1)

    def get_latest_frame(self):
        with self.lock:
            if not self.has_frame:
                return False, None
            return True, self.latest_frame.copy()


def make_flask_app(cam: CameraServer) -> Flask:
    """สร้าง Flask app + encode worker สำหรับกล้องแต่ละตัว"""
    app        = Flask(cam.name)
    enc_params = [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY]
    q          = queue.Queue(maxsize=2)

    def _encode_worker():
        interval = 1.0 / FPS
        while True:
            t0 = time.time()
            ok, frame = cam.get_latest_frame()
            if not ok:
                time.sleep(0.005)
                continue
            ret, buf = cv2.imencode('.jpg', frame, enc_params)
            if not ret:
                continue
            data = buf.tobytes()
            if q.full():
                try:
                    q.get_nowait()
                except queue.Empty:
                    pass
            try:
                q.put_nowait(data)
            except queue.Full:
                pass
            elapsed = time.time() - t0
            if elapsed < interval:
                time.sleep(interval - elapsed)

    threading.Thread(target=_encode_worker, daemon=True).start()

    def _generate():
        while True:
            try:
                data = q.get(timeout=0.5)
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n'
                       + data + b'\r\n')
            except queue.Empty:
                yield b'--frame\r\nContent-Type: text/plain\r\n\r\n\r\n'

    @app.route('/stream')
    def stream():
        return Response(
            _generate(),
            mimetype='multipart/x-mixed-replace; boundary=frame',
            headers={
                'Cache-Control':     'no-cache, no-store, must-revalidate',
                'X-Accel-Buffering': 'no',
                'Pragma':            'no-cache',
                'Expires':           '0',
                'Connection':        'keep-alive',
            }
        )

    @app.route('/health')
    def health():
        return {
            "status": "ok" if cam.has_frame else "waiting",
            "camera": cam.index,
            "name":   cam.name,
        }

    return app


def run_app(app: Flask, port: int):
    import logging
    logging.getLogger('werkzeug').setLevel(logging.ERROR)
    app.run(host='0.0.0.0', port=port,
            threaded=True, debug=False, use_reloader=False)


if __name__ == '__main__':
    cam1 = CameraServer(CAMERA_INDEX_1, name="CAM1-AprilTag")
    cam2 = CameraServer(CAMERA_INDEX_2, name="CAM2-Cabbage")

    app1 = make_flask_app(cam1)
    app2 = make_flask_app(cam2)

    print("=" * 50)
    print(f"  CAM1 (AprilTag) index={CAMERA_INDEX_1} → :5000/stream")
    print(f"  CAM2 (Cabbage)  index={CAMERA_INDEX_2} → :5001/stream")
    print("=" * 50)

    t1 = threading.Thread(target=run_app, args=(app1, 5000), daemon=True)
    t2 = threading.Thread(target=run_app, args=(app2, 5001), daemon=True)
    t1.start()
    t2.start()

    try:
        t1.join()
        t2.join()
    except KeyboardInterrupt:
        print("\n[STOP]")