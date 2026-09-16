"""Python 3.8 stdlib endpoint: ONLY SO-101 USB, never Unitree motion APIs."""
import argparse
import json
import math
import os
import select
import signal
import sys
import termios
import threading
import time


def packet(mid, instruction, params):
    body = [mid, len(params) + 2, instruction] + list(params)
    return bytes([255, 255] + body + [(~sum(body)) & 255])


class Bus:
    def __init__(self, port):
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        import fcntl
        fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        attr = termios.tcgetattr(self.fd)
        attr[0] = attr[1] = attr[3] = 0
        attr[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        attr[4] = attr[5] = termios.B1000000
        attr[6][termios.VMIN] = attr[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, attr)

    def send(self, data):
        if os.write(self.fd, data) != len(data):
            raise OSError('Incomplete USB write')

    def read(self, mid, address, count):
        termios.tcflush(self.fd, termios.TCIFLUSH)
        self.send(packet(mid, 2, [address, count]))
        data = b''
        deadline = time.monotonic() + .06
        while len(data) < count + 6:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(f'USB read timeout, motor {mid}')
            if select.select([self.fd], [], [], remaining)[0]:
                data += os.read(self.fd, 256)
        if (len(data) != count + 6 or data[:4] != bytes([255, 255, mid, count + 2])
                or sum(data[2:]) & 255 != 255 or data[4]):
            raise OSError('Invalid motor response: ' + data.hex())
        return int.from_bytes(data[5:-1], 'little')

    def sync(self, address, count, values):
        params = [address, count]
        for mid, value in sorted(values.items()):
            params += [mid] + list(int(value).to_bytes(count, 'little'))
        self.send(packet(254, 131, params))

    def close(self):
        os.close(self.fd)


class Arm:
    def __init__(self, bus, calibration, watchdog=.5):
        self.bus = bus
        self.cal = {int(k): v for k, v in calibration.items()}
        if set(self.cal) != set(range(1, 7)):
            raise ValueError('Exactly six calibrated motors required')
        self.watchdog = watchdog
        self.lock = threading.RLock()
        self.armed = False
        self.touched = False
        self.last = 0.
        self.sampled = None
        self.hold_target = None
        self.closed = threading.Event()
        self.thread = threading.Thread(target=self.watch, daemon=True)
        self.thread.start()

    def positions(self):
        result = {mid: self.bus.read(mid, 56, 2) for mid in self.cal}
        if any(not 0 <= val <= 4095 for val in result.values()):
            raise ValueError('Invalid signed/out-of-range servo position')
        return result

    def sample(self):
        positions = self.positions()
        self.sampled = time.monotonic()
        return {'positions': positions, 'sampled': self.sampled, 'armed': self.armed}

    def inspect(self):
        result = {}
        for mid, cal in self.cal.items():
            offset = self.bus.read(mid, 31, 2)
            offset = -(offset & 2047) if offset & 2048 else offset
            row = {'model': self.bus.read(mid, 3, 2), 'min': self.bus.read(mid, 9, 2),
                   'max': self.bus.read(mid, 11, 2), 'offset': offset,
                   'mode': self.bus.read(mid, 33, 1), 'torque': self.bus.read(mid, 40, 1),
                   'pid': [self.bus.read(mid, addr, 1) for addr in (21, 23, 22)]}
            if (row['model'] != 777 or row['min'] != cal['range_min']
                    or row['max'] != cal['range_max'] or offset != cal['homing_offset']
                    or row['mode'] != 0):
                raise ValueError(f'Motor/calibration/mode mismatch: {mid} {row}')
            result[mid] = row
        return {'motors': result, **self.sample()}

    def stop(self):
        # Position servos retain their last bounded goal locally. Never remove
        # support from a gravity-loaded arm on network loss or process shutdown.
        self.armed = False

    def handle(self, request):
        with self.lock:
            op = request['op']
            if op == 'inspect':
                return self.inspect()
            if op == 'sample':
                return self.sample()
            if op == 'stop':
                self.stop()
                return {'stopped': True}
            if op not in ('arm', 'action', 'hold'):
                raise ValueError('Unknown operation')
            now = time.monotonic()
            if self.sampled is None or request['sampled'] != self.sampled or now - self.sampled > .25:
                raise TimeoutError('Stale command; restart teleop')
            if op == 'arm':
                if self.armed or self.touched:
                    raise RuntimeError('Arming only once per session')
                torques = [self.bus.read(mid, 40, 1) for mid in self.cal]
                if any(torques) and not (all(v == 1 for v in torques) and request.get('accept_powered', False)):
                    raise RuntimeError('Arm already powered; refusing takeover')
                current = self.positions()
                self.bus.sync(42, 2, current)
                self.touched = True
                self.bus.sync(40, 1, dict.fromkeys(self.cal, 1))
                if any(self.bus.read(mid, 40, 1) != 1 for mid in self.cal):
                    raise RuntimeError('Torque-on verification failed')
                self.target = current
                self.last = time.monotonic()
                self.armed = True
                return self.sample()
            if not self.armed:
                raise RuntimeError('Watchdog latched; restart teleop')
            if op == 'hold':
                if self.hold_target is None:
                    self.hold_target = self.positions()
                self.bus.sync(42, 2, self.hold_target)
                self.target = dict(self.hold_target)
                self.last = time.monotonic()
                return {**self.sample(), 'sent': dict(self.target)}
            self.hold_target = None
            goal = {int(k): v for k, v in request['positions'].items()}
            if set(goal) != set(self.cal) or any(not isinstance(v, (int, float)) or not math.isfinite(v)
                                                for v in goal.values()):
                raise ValueError('Invalid target')
            # Rate limit commanded targets to 60 deg/s; gripper 100 percentage points/s.
            dt = min(now - self.last, .1)
            sent = {}
            for mid, cal in self.cal.items():
                limit = (4095 * 60 / 360 if mid != 6 else cal['range_max'] - cal['range_min']) * dt
                value = max(cal['range_min'], min(cal['range_max'], goal[mid]))
                sent[mid] = int(round(max(self.target[mid] - limit, min(self.target[mid] + limit, value))))
            self.bus.sync(42, 2, sent)
            self.target = sent
            self.last = time.monotonic()
            return {**self.sample(), 'sent': sent}

    def watch(self):
        while not self.closed.wait(.02):
            with self.lock:
                if self.armed and time.monotonic() - self.last > self.watchdog:
                    try:
                        self.stop()
                        print('WATCHDOG: arm command stream latched; torque and last goal retained', file=sys.stderr, flush=True)
                    except Exception as exc:
                        print('WATCHDOG STOP FAILED: ' + str(exc), file=sys.stderr, flush=True)

    def close(self):
        self.closed.set()
        try:
            with self.lock:
                self.stop()
        finally:
            self.thread.join(timeout=1)
            self.bus.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', required=True)
    parser.add_argument('--calibration', required=True)
    args = parser.parse_args()
    arm = Arm(Bus(args.port), json.loads(args.calibration))
    signal.signal(signal.SIGTERM, lambda *args: sys.exit(0))
    previous = 0
    try:
        print(json.dumps({'ready': True}), flush=True)
        for line in sys.stdin:
            try:
                request = json.loads(line)
                if request['id'] <= previous:
                    raise ValueError('Repeated command ID')
                previous = request['id']
                reply = arm.handle(request)
                print(json.dumps({'id': previous, 'result': reply}), flush=True)
            except Exception as exc:
                with arm.lock:
                    arm.stop()
                print(json.dumps({'id': previous, 'error': str(exc)}), flush=True)
                break
    finally:
        arm.close()


if __name__ == '__main__':
    main()
