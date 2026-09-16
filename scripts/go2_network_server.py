"""Ephemeral Python 3.8 Go2 endpoint. JSON lines over SSH; no dataset or files onboard."""
import argparse
import base64
import json
import math
import os
import select
import signal
import subprocess
import sys
import threading
import time


class MotionGuard:
    """Independent onboard deadline, freshness checks and a latched stop."""
    def __init__(self, send, stop, limits, timeout=0.25):
        self.send = send
        self.stop_fn = stop
        self.limits = limits
        self.timeout = timeout
        self.lock = threading.RLock()
        self.last_command = None
        self.latched = False
        self.closed = threading.Event()
        self.thread = threading.Thread(target=self.watch, daemon=True)
        self.thread.start()

    def command(self, values, sampled_at, fresh_at):
        with self.lock:
            now = time.monotonic()
            if len(values) != 3 or not all(math.isfinite(v) for v in values):
                self.stop()
                raise ValueError("Invalid command")
            if min(sampled_at, fresh_at) < now - self.timeout:
                self.stop()
                raise TimeoutError("Expired observation/command; release controls")
            values = [max(-limit, min(limit, float(v))) for v, limit in zip(values, self.limits)]  # noqa: B905 -- onboard Python 3.8
            if self.latched and any(values):
                raise RuntimeError("Watchdog stop latched; release movement keys to rearm")
            try:
                self.send(*values)
            except BaseException:
                self.stop()
                raise
            self.last_command = time.monotonic()
            if not any(values):
                self.latched = False
            return values

    def stop(self):
        with self.lock:
            self.last_command = None
            self.latched = True
            self.stop_fn()

    def watch(self):
        while not self.closed.wait(0.02):
            with self.lock:
                if self.last_command is not None and time.monotonic() - self.last_command > self.timeout:
                    try:
                        self.stop()
                    except Exception as exc:
                        print("WATCHDOG STOP FAILED: " + str(exc), file=sys.stderr, flush=True)

    def close(self):
        self.closed.set()
        try:
            self.stop()
        finally:
            self.thread.join(timeout=1)


class Hardware:
    def __init__(self, cfg, mock=False, readonly=False):
        import cv2
        import numpy as np
        self.cv2, self.np = cv2, np
        self.cfg, self.mock, self.readonly = cfg, mock, readonly
        self.lock = threading.Lock()
        self.state = None
        self.frame = None
        self.error = None
        self.closed = threading.Event()
        self.history = []
        self.sub = self.camera = self.client = self.guard = None
        self.thread = None
        self.frame_id = 0

    def on_state(self, msg):
        with self.lock:
            self.state = {"values": [float(msg.velocity[0]), float(msg.velocity[1]), float(msg.yaw_speed)]
                          + [float(x) for x in msg.imu_state.rpy],
                          "received": time.monotonic(),
                          "source": float(msg.stamp.sec) + float(msg.stamp.nanosec) * 1e-9}

    def connect(self):
        quality = self.cfg.get("jpeg_quality", 90)
        if not isinstance(quality, int) or not 60 <= quality <= 95:
            raise ValueError("jpeg_quality must be an integer from 60 to 95")
        if not self.mock:
            from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
            from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
            ChannelFactoryInitialize(self.cfg.get("domain_id", 0), self.cfg["network_interface"])
            self.sub = ChannelSubscriber(self.cfg["state_topic"], SportModeState_)
            self.sub.Init(self.on_state, 10)
            properties = subprocess.check_output(
                ["udevadm", "info", "--query=property", "--name=" + self.cfg["device"]], text=True)
            if "ID_SERIAL_SHORT=" + self.cfg["usb_serial"] not in properties.splitlines():
                raise RuntimeError("Camera device does not match requested USB serial")
            devices = subprocess.check_output(["rs-enumerate-devices", "-s"], text=True)
            if self.cfg["serial"] not in devices:
                raise RuntimeError("Requested RealSense SDK serial is not present")
            self.camera = self.cv2.VideoCapture(self.cfg["device"], self.cv2.CAP_V4L2)
            for prop, value in [(self.cv2.CAP_PROP_FRAME_WIDTH, 640), (self.cv2.CAP_PROP_FRAME_HEIGHT, 480),
                                (self.cv2.CAP_PROP_FPS, 30), (self.cv2.CAP_PROP_BUFFERSIZE, 1)]:
                self.camera.set(prop, value)
            if not self.camera.isOpened():
                raise RuntimeError("Cannot open onboard D435i RGB")
            if not self.readonly:
                from unitree_sdk2py.go2.sport.sport_client import SportClient
                self.client = SportClient()
                self.client.SetTimeout(0.1)
                self.client.Init()
        self.thread = threading.Thread(target=self.capture, daemon=True)
        self.thread.start()
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            with self.lock:
                ready = self.frame is not None and self.state is not None
            if ready:
                break
            if self.error:
                raise RuntimeError(self.error)
            time.sleep(0.02)
        else:
            raise TimeoutError("No camera/state before startup deadline")
        self.guard = MotionGuard(self.send, self.stop, self.cfg["limits"], self.cfg["watchdog"])

    def capture(self):
        try:
            while not self.closed.is_set():
                start = time.monotonic()
                if self.mock:
                    frame = self.np.zeros((480, 640, 3), dtype=self.np.uint8)
                    frame[:, :, 1] = self.frame_id % 255
                    with self.lock:
                        self.state = {"values": [0.] * 6, "received": start, "source": start}
                else:
                    ok, frame = self.camera.read()
                    if not ok:
                        raise RuntimeError("RealSense disconnected")
                stamp = time.monotonic()
                if frame.shape != (480, 640, 3):
                    raise ValueError("Expected RGB 640x480")
                ok, data = self.cv2.imencode(
                    ".jpg", frame, [self.cv2.IMWRITE_JPEG_QUALITY, self.cfg.get("jpeg_quality", 90)]
                )
                if not ok:
                    raise RuntimeError("Camera JPEG encoding failed")
                with self.lock:
                    self.frame_id += 1
                    self.frame = {"jpeg": base64.b64encode(data.tobytes()).decode("ascii"),
                                  "received": stamp, "frame_id": self.frame_id}
                if self.mock:
                    self.closed.wait(max(0, start + 1 / 30 - time.monotonic()))
        except BaseException as exc:
            self.error = str(exc)
            if self.guard:
                self.guard.stop()

    def sample(self):
        with self.lock:
            state, frame = self.state, self.frame
        now = time.monotonic()
        if self.error or not state or not frame:
            raise RuntimeError(self.error or "Missing streams")
        if len(state["values"]) != 6 or not all(math.isfinite(v) for v in state["values"]):
            raise ValueError("Invalid state values")
        if now - state["received"] > self.cfg["watchdog"] or now - frame["received"] > 0.2:
            raise TimeoutError("Stale onboard camera/state")
        return {"state": state, "camera": frame, "sampled_at": now}

    def send(self, vx, vy, wz):
        if self.readonly:
            if any((vx, vy, wz)):
                raise PermissionError("Read-only endpoint rejects motion")
            return  # No RPC at all in read-only mode, including zero commands.
        if self.mock:
            self.history.append([vx, vy, wz])
        else:
            result = self.client.Move(vx, vy, wz)
            if result != 0:
                raise RuntimeError("SportClient.Move returned " + str(result))

    def stop(self):
        if self.readonly:
            return
        if self.mock:
            self.history.append([0., 0., 0.])
        elif self.client:
            failures = []
            for call in (lambda: self.client.Move(0., 0., 0.), self.client.StopMove):
                try:
                    result = call()
                    if result != 0:
                        failures.append(result)
                except Exception as exc:
                    failures.append(str(exc))
            if len(failures) == 2:
                raise RuntimeError("Both stop RPCs failed: " + repr(failures))

    def close(self):
        try:
            if self.guard:
                self.guard.close()
            else:
                self.stop()
        finally:
            self.closed.set()
            if self.thread:
                self.thread.join(timeout=1)
            if self.camera:
                self.camera.release()
            if self.sub:
                self.sub.Close()


def serve(cfg, mock=False, readonly=False):
    output = sys.stdout
    sys.stdout = sys.stderr  # SDK diagnostic prints must not corrupt the protocol.
    hardware = Hardware(cfg, mock=mock, readonly=readonly)

    def reply(value):
        output.write(json.dumps(value, allow_nan=False) + "\n")
        output.flush()

    def interrupt(*_):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupt)
    latest = None
    previous_id = 0
    delays_left = cfg.get("mock_delay_count", 1) if mock else 0
    delay_after = cfg.get("mock_delay_after", 0) if mock else 0
    matching_operations = 0
    disconnects_left = cfg.get("mock_disconnect_count", 0) if mock else 0
    disconnect_after = cfg.get("mock_disconnect_after", 0) if mock else 0
    matching_disconnect_operations = 0
    buffer = b""
    try:
        hardware.connect()
        reply({"ready": True, "protocol": 1, "readonly": readonly, "mock": mock})
        while True:
            readable, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not readable:
                continue
            data = os.read(sys.stdin.fileno(), 4096)
            if not data:
                break
            buffer += data
            if len(buffer) > 16384:
                raise ValueError("Request too large")
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                request = json.loads(line)
                request_id = request["id"]
                if request_id <= previous_id:
                    raise ValueError("Replayed request")
                previous_id = request_id
                op = request["op"]
                try:
                    if op == "sample":
                        latest = hardware.sample()
                        result = latest
                    elif op == "action":
                        if latest is None or request["sampled_at"] != latest["sampled_at"]:
                            raise ValueError("Action must reference latest observation")
                        fresh = hardware.sample()
                        sent = hardware.guard.command(request["values"], latest["sampled_at"],
                            min(fresh["state"]["received"], fresh["camera"]["received"]))
                        result = {"sent": sent, "sent_monotonic": time.monotonic()}
                    elif op == "stop":
                        hardware.guard.stop()
                        result = {"stopped": True}
                    elif op == "status" and mock:
                        result = {"history": hardware.history, "latched": hardware.guard.latched}
                    elif op == "close":
                        hardware.guard.stop()
                        reply({"id": request_id, "result": {"closed": True}})
                        return
                    else:
                        raise ValueError("Unknown operation")
                    if mock and op == cfg.get("mock_disconnect_op"):
                        if disconnects_left > 0 and matching_disconnect_operations >= disconnect_after:
                            disconnects_left -= 1
                            hardware.guard.stop()
                            return
                        matching_disconnect_operations += 1
                    if mock and op == cfg.get("mock_delay_op"):
                        if delays_left > 0 and matching_operations >= delay_after:
                            delays_left -= 1
                            time.sleep(cfg.get("mock_delay_s", 0.35))
                        matching_operations += 1
                    reply({"id": request_id, "result": result})
                except Exception as exc:
                    hardware.guard.stop()
                    reply({"id": request_id, "error": str(exc)})
    finally:
        hardware.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--mock", action="store_true")
    parser.add_argument("--readonly", action="store_true")
    args = parser.parse_args()
    serve(json.loads(args.config), args.mock, args.readonly)
