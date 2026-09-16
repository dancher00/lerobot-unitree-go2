"""Laptop backend and remote camera for the existing UnitreeGo2 LeRobot adapter."""
import base64
import contextlib
import json
import os
import select
import shlex
import subprocess
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np

from lerobot_robot_unitree_go2 import Go2State, UnitreeGo2, UnitreeGo2Config
from lerobot_robot_unitree_go2.unitree.backend import Go2Backend


class StaleNetworkFrame(TimeoutError):
    """A complete reply arrived, but its image is too old; protocol remains synchronized."""


class StoppedNetworkTimeout(TimeoutError):
    """A timed-out operation was drained and an explicit stop was acknowledged."""


class EndpointDisconnected(ConnectionError):
    """The SSH command channel ended before a complete protocol reply arrived."""


class Connection:
    def __init__(self, cfg, mock=False, readonly=False, log=None, endpoint_stderr_path=None,
                 endpoint_source=None):
        self.cfg, self.mock, self.readonly = cfg, mock, readonly
        self.process = None
        self.buffer = b""
        self.counter = 0
        self.lock = threading.RLock()
        self.log = log
        self.last_latency = 0.
        self.endpoint_stderr_path = Path(endpoint_stderr_path) if endpoint_stderr_path else None
        self.endpoint_stderr = None
        self.endpoint_source = endpoint_source

    def connect(self):
        network = self.cfg["network"]
        endpoint = {"network_interface": network["interface"], "state_topic": self.cfg["state_topic"],
                    "domain_id": self.cfg["domain_id"], "device": network["camera_device"],
                    "jpeg_quality": network.get("jpeg_quality", 90),
                    "serial": os.environ.get("CAMERA_SERIAL", self.cfg["camera"]["serial"]),
                    "usb_serial": self.cfg["camera"]["usb_serial"],
                    "limits": [self.cfg["safety"]["max_" + axis] for axis in ("vx", "vy", "wz")],
                    "watchdog": self.cfg["safety"]["watchdog_timeout_s"]}
        if 'wrist_device' in network:
            endpoint['wrist_device'] = network['wrist_device']
        if self.mock:
            endpoint.update({key: network[key] for key in
                             ("mock_delay_op", "mock_delay_s", "mock_delay_count", "mock_delay_after",
                              "mock_disconnect_op", "mock_disconnect_count", "mock_disconnect_after")
                             if key in network})
            if network.get("mock_disconnect_count", 0) > 0:
                # Fault injection is one-shot across a reconnect within this laptop process.
                network["mock_disconnect_count"] -= 1
        script = Path(__file__).with_name("go2_network_server.py")
        args = ["--config", json.dumps(endpoint)]
        if self.readonly:
            args.append("--readonly")
        if self.mock:
            command = ([sys.executable, '-u', '-c', self.endpoint_source.decode(), *args, '--mock']
                       if self.endpoint_source else [sys.executable, "-u", str(script), *args, "--mock"])
        else:
            encoded = base64.b64encode(self.endpoint_source or script.read_bytes()).decode("ascii")
            bootstrap = "import base64;exec(compile(base64.b64decode(" + repr(encoded) + "),'<go2-endpoint>','exec'))"
            remote = ["env", "PYTHONDONTWRITEBYTECODE=1", "PYTHONPATH=" + network["pythonpath"],
                      network["python"], "-u", "-c", bootstrap, *args]
            command = ["ssh", "-T", "-o", "StrictHostKeyChecking=yes", "-o", "ConnectTimeout=5",
                       "-o", "IPQoS=lowdelay", "-o", "Compression=no",
                       "-o", "ServerAliveInterval=2", "-o", "ServerAliveCountMax=2",
                       os.environ.get("GO2_HOST", network["host"]), shlex.join(remote)]
            if network.get('ssh_control_path'):
                command[1:1] = ['-S', network['ssh_control_path'], '-o', 'ControlMaster=auto',
                                '-o', 'ControlPersist=120']
        if self.endpoint_stderr_path:
            self.endpoint_stderr_path.parent.mkdir(parents=True, exist_ok=True)
            self.endpoint_stderr = self.endpoint_stderr_path.open("ab", buffering=0)
        self.process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                        stderr=self.endpoint_stderr, bufsize=0)
        try:
            ready = self.read(90)
            if ready != {"ready": True, "protocol": 1, "readonly": self.readonly, "mock": self.mock}:
                raise RuntimeError("Unexpected endpoint handshake: " + repr(ready))
        except BaseException:
            self.close()
            raise

    def read(self, timeout):
        deadline = time.monotonic() + timeout
        while b"\n" not in self.buffer:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("Go2 network response deadline exceeded")
            readable, _, _ = select.select([self.process.stdout], [], [], remaining)
            if not readable:
                continue
            data = os.read(self.process.stdout.fileno(), 65536)
            if not data:
                returncode = self.process.poll()
                detail = "running" if returncode is None else "exit " + str(returncode)
                location = ("; details: " + str(self.endpoint_stderr_path)) if self.endpoint_stderr_path else ""
                raise EndpointDisconnected(f"Go2 SSH endpoint disconnected ({detail}{location})")
            self.buffer += data
            if len(self.buffer) > 4 * 1024 * 1024:
                raise ValueError("Oversized network frame")
        line, self.buffer = self.buffer.split(b"\n", 1)
        return json.loads(line)

    def rpc(self, op, **fields):
        with self.lock:
            if self.process is None:
                raise ConnectionError("Go2 connection is closed")
            self.counter += 1
            started = time.monotonic()
            packet = {"op": op, "id": self.counter, **fields}
            try:
                self.process.stdin.write((json.dumps(packet, allow_nan=False) + "\n").encode())
                result = self.read(self.cfg["network"]["response_timeout_s"])
            except TimeoutError as exc:
                self.recover_stop(packet["id"], op)
                raise StoppedNetworkTimeout(f"Go2 {op} timed out; late reply drained and stop acknowledged") from exc
            except (OSError, ValueError):
                self.close()
                raise
            self.last_latency = time.monotonic() - started
            if result.get("id") != self.counter:
                self.close()
                raise RuntimeError("Out-of-order acknowledgment")
            if "error" in result:
                raise RuntimeError("Go2 endpoint: " + result["error"])
            if self.log:
                payload = result["result"]
                if op == "sample":
                    payload = {k: ({a: b for a, b in v.items() if a != 'jpeg'}
                                   if k in ('camera', 'wrist') else v) for k, v in payload.items()}
                self.log.write(json.dumps({"op": op, "request": packet, "result": payload,
                                           "laptop_monotonic": started, "round_trip_s": self.last_latency}) + "\n")
                self.log.flush()
            return result["result"]

    def recover_stop(self, expired_id, expired_op):
        """Send stop immediately, then drain only the known expired request within a fixed deadline."""
        self.counter += 1
        stop_id = self.counter
        deadline = time.monotonic() + 1.0
        pending = {expired_id}
        try:
            self.process.stdin.write((json.dumps({"id": stop_id, "op": "stop"}) + "\n").encode())
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("Stop acknowledgment deadline exceeded")
                reply = self.read(remaining)
                reply_id = reply.get("id")
                if reply_id in pending:
                    pending.remove(reply_id)
                    continue  # This reply belongs to the expired operation, never to stop.
                if reply_id != stop_id or reply.get("result") != {"stopped": True}:
                    raise RuntimeError("Invalid stop acknowledgment during recovery")
                if self.log:
                    self.log.write(json.dumps({"op": "timeout_recovery", "expired_id": expired_id,
                                               "expired_op": expired_op, "stop_id": stop_id,
                                               "stop_acknowledged": True, "laptop_monotonic": time.monotonic()}) + "\n")
                    self.log.flush()
                return
        except (OSError, ValueError, RuntimeError) as exc:
            self.close()
            raise ConnectionError("Go2 connection closed: stop could not be confirmed; onboard watchdog remains the fallback") from exc

    def close(self):
        with self.lock:
            if self.process:
                # Closing stdin triggers onboard finally; watchdog remains independent of EOF delivery.
                with contextlib.suppress(BrokenPipeError, OSError):
                    self.process.stdin.close()
                try:
                    self.process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self.process.terminate()
                    try:
                        self.process.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        self.process.kill()
                        self.process.wait(timeout=2)
                self.process.stdout.close()
                self.process = None
            if self.endpoint_stderr:
                self.endpoint_stderr.close()
                self.endpoint_stderr = None


class NetworkBackend(Go2Backend):
    def __init__(self, connection):
        self.connection = connection
        self.sample = None
        self.sample_started = 0.
        self.last_sent = None

    @property
    def is_connected(self):
        return self.connection.process is not None and self.connection.process.poll() is None

    def connect(self):
        self.connection.connect()

    def read_state(self):
        self.sample_started = time.monotonic()
        sample = self.connection.rpc("sample")
        if self.sample is not None and sample["state"]["source"] < self.sample["state"]["source"]:
            raise RuntimeError("Go2 source timestamp moved backwards")
        self.sample = sample
        state = self.sample["state"]
        age = self.sample["sampled_at"] - state["received"]
        # A conservative local timestamp includes the full network request interval.
        return Go2State(**dict(zip(("vx", "vy", "wz", "roll", "pitch", "yaw"), state["values"], strict=True)),
                        received_monotonic_s=self.sample_started - age,
                        source_timestamp_s=state["source"])

    def send_velocity(self, vx, vy, wz):
        if self.sample is None:
            raise RuntimeError("Observation required before command")
        values = [vx, vy, wz]
        if not all(np.isfinite(values)):
            raise ValueError("Non-finite motion command")
        ack = self.connection.rpc("action", sampled_at=self.sample["sampled_at"], values=values)
        if ack["sent"] != values:
            raise RuntimeError("Endpoint command differs from recorded command")
        self.last_sent = ack

    def stop(self):
        if self.is_connected:
            self.connection.rpc("stop")

    def disconnect(self):
        try:
            self.stop()
        finally:
            self.connection.close()


class NetworkCamera:
    width, height, fps = 640, 480, 30
    use_rgb, use_depth = True, False

    def __init__(self, backend, sample_key='camera'):
        self.backend = backend
        self.sample_key = sample_key
        self.is_connected = False
        self.latest_timestamp = None

    def connect(self):
        self.is_connected = True

    def read_latest(self, max_age_ms=200):
        sample = self.backend.sample
        camera = sample[self.sample_key]
        age = sample["sampled_at"] - camera["received"]
        self.latest_timestamp = self.backend.sample_started - age
        age_s = time.monotonic() - self.latest_timestamp
        if age_s > max_age_ms / 1000:
            raise StaleNetworkFrame(f"Network camera frame is stale ({age_s * 1000:.0f} ms > {max_age_ms} ms)")
        bgr = cv2.imdecode(np.frombuffer(base64.b64decode(camera["jpeg"], validate=True), dtype=np.uint8), cv2.IMREAD_COLOR)
        if bgr is None or bgr.shape != (480, 640, 3):
            raise ValueError("Invalid RGB frame received")
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

    def disconnect(self):
        self.is_connected = False


def make_network_robot(cfg, mock=False, readonly=False, log=None, raw_log_path=None,
                       endpoint_stderr_path=None, endpoint_source=None):
    connection = Connection(cfg, mock=mock, readonly=readonly, log=log,
                            endpoint_stderr_path=endpoint_stderr_path, endpoint_source=endpoint_source)
    backend = NetworkBackend(connection)
    # Construction-only mock config avoids opening a local USB camera. The explicit backend
    # and camera below perform all I/O; this does not enable simulated hardware on real runs.
    config = UnitreeGo2Config(mock=True, mock_camera=False, cameras={},
                             control_frequency=cfg["control_frequency"], raw_log_path=raw_log_path, **cfg["safety"])
    robot = UnitreeGo2(config, backend=backend)
    robot.cameras = {"front": NetworkCamera(backend)}
    if 'wrist_device' in cfg['network']:
        robot.cameras['wrist'] = NetworkCamera(backend, 'wrist')
    return robot
