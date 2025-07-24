# zed_mjpeg_stream.py
import threading
import time
import cv2
import pyzed.sl as sl

class ZEDCameraStream:
    def __init__(self):
        self.zed = sl.Camera()
        init_params = sl.InitParameters(camera_resolution=sl.RESOLUTION.HD720, depth_mode=sl.DEPTH_MODE.NONE)
        status = self.zed.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"❌ Failed to open ZED camera: {status}")
        
        self.image = sl.Mat()
        self.frame = None
        self.running = True
        self.lock = threading.Lock()

        # Start capture thread
        self.thread = threading.Thread(target=self.update_frames)
        self.thread.daemon = True
        self.thread.start()

    def update_frames(self):
        while self.running:
            if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
                self.zed.retrieve_image(self.image, sl.VIEW.LEFT)
                frame = self.image.get_data()
                if frame is not None:
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
                    with self.lock:
                        self.frame = frame
            time.sleep(0.01)

    def get_frame(self):
        with self.lock:
            return self.frame.copy() if self.frame is not None else None

    def stop(self):
        self.running = False
        self.thread.join()
        self.zed.close()

stream_instance = None

def start_zed_stream():
    global stream_instance
    if stream_instance is None:
        stream_instance = ZEDCameraStream()

def stop_zed_stream():
    global stream_instance
    if stream_instance:
        stream_instance.stop()
        stream_instance = None

from gevent import sleep  # make sure this is imported

def generate_mjpeg():
    global stream_instance
    while True:
        if stream_instance is None:
            print("⏳ Stream not ready yet...")
            sleep(0.1)
            continue
        frame = stream_instance.get_frame()
        if frame is None:
            print("❗ No frame captured...")
            sleep(0.1)
            continue
        ret, buffer = cv2.imencode('.jpg', frame)
        if not ret:
            sleep(0.05)
            continue
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
        sleep(0.03)  # ~30fps


