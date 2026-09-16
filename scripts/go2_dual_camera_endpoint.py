"""Extension loaded alongside go2_network_server in memory; no onboard install."""
import base64
import threading
import time

import go2_stream


class DualHardware(go2_stream.Hardware):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.wrist_camera = None
        self.wrist_thread = None
        self.wrist_frame = None
        self.wrist_id = 0

    def connect(self):
        super().connect()
        if not self.mock:
            self.wrist_camera = self.cv2.VideoCapture(self.cfg['wrist_device'], self.cv2.CAP_V4L2)
            for prop, value in ((self.cv2.CAP_PROP_FOURCC, self.cv2.VideoWriter_fourcc(*'MJPG')),
                                (self.cv2.CAP_PROP_FRAME_WIDTH, 640), (self.cv2.CAP_PROP_FRAME_HEIGHT, 480),
                                (self.cv2.CAP_PROP_FPS, 30), (self.cv2.CAP_PROP_BUFFERSIZE, 1)):
                self.wrist_camera.set(prop, value)
            if not self.wrist_camera.isOpened():
                raise RuntimeError('Cannot open wrist camera')
        self.wrist_thread = threading.Thread(target=self.capture_wrist, daemon=True)
        self.wrist_thread.start()
        deadline = time.monotonic() + 5
        while self.wrist_frame is None:
            if self.error or time.monotonic() > deadline:
                raise RuntimeError(self.error or 'No wrist frames')
            time.sleep(.01)

    def capture_wrist(self):
        try:
            while not self.closed.is_set():
                start = time.monotonic()
                if self.mock:
                    frame = self.np.zeros((480, 640, 3), dtype=self.np.uint8)
                    frame[:, :, 2] = self.wrist_id % 255
                else:
                    ok, frame = self.wrist_camera.read()
                    if not ok:
                        raise RuntimeError('Wrist camera disconnected')
                stamp = time.monotonic()
                if frame.shape != (480, 640, 3):
                    raise RuntimeError('Expected wrist RGB 640x480')
                ok, data = self.cv2.imencode('.jpg', frame, [self.cv2.IMWRITE_JPEG_QUALITY, self.cfg.get('jpeg_quality', 90)])
                if not ok:
                    raise RuntimeError('Wrist encoding failed')
                with self.lock:
                    self.wrist_id += 1
                    self.wrist_frame = dict(jpeg=base64.b64encode(data.tobytes()).decode('ascii'),
                                           received=stamp, frame_id=self.wrist_id)
                if self.mock:
                    self.closed.wait(max(0, start + 1 / 30 - time.monotonic()))
        except Exception as exc:
            self.error = str(exc)
            if self.guard:
                self.guard.stop()

    def sample(self):
        result = super().sample()
        with self.lock:
            frame = self.wrist_frame
        if frame is None or time.monotonic() - frame['received'] > .2:
            raise TimeoutError('Stale wrist camera')
        result['wrist'] = frame
        return result

    def close(self):
        try:
            super().close()
        finally:
            if self.wrist_thread:
                self.wrist_thread.join(timeout=1)
            if self.wrist_camera:
                self.wrist_camera.release()


go2_stream.Hardware = DualHardware
