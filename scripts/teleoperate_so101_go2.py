"""Local SO-101 leader -> SSH -> SO-101 follower USB on Go2 (no leg control)."""
import argparse
import base64
import contextlib
import json
import os
import select
import shlex
import signal
import subprocess
import sys
import tempfile
import termios
import time
import tty
from dataclasses import asdict
from pathlib import Path

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig

BASE = Path(__file__).resolve().parents[1]


class Remote:
    def __init__(self, args, calibration, run):
        code = base64.b64encode(Path(__file__).with_name('so101_remote_endpoint.py').read_bytes()).decode()
        bootstrap = 'import base64;exec(compile(base64.b64decode(' + repr(code) + "),'<so101-endpoint>','exec'))"
        command = ['ssh', '-T', '-o', 'StrictHostKeyChecking=yes', '-o', 'ConnectTimeout=5',
                   '-o', 'ServerAliveInterval=2', '-o', 'ServerAliveCountMax=2']
        if args.ssh_control_path:
            command += ['-S', args.ssh_control_path, '-o', 'ControlMaster=auto', '-o', 'ControlPersist=120']
        command += [args.host, shlex.join(['python3', '-u', '-c', bootstrap, '--port', args.follower_port,
                                         '--calibration', json.dumps(calibration)])]
        self.stderr = (run / 'endpoint.stderr.log').open('xb')
        self.log = (run / 'rpc.jsonl').open('x')
        self.process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                        stderr=self.stderr, bufsize=0)
        self.buffer = b''
        self.counter = 0
        try:
            if self.read(90) != {'ready': True}:
                raise RuntimeError('Invalid handshake')
        except BaseException:
            self.close()
            raise

    def read(self, timeout):
        deadline = time.monotonic() + timeout
        while b'\n' not in self.buffer:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError('SSH reply timeout; arm retains torque and last goal')
            if select.select([self.process.stdout], [], [], remaining)[0]:
                data = os.read(self.process.stdout.fileno(), 65536)
                if not data:
                    raise ConnectionError('SO-101 endpoint disconnected; see endpoint.stderr.log')
                self.buffer += data
                if len(self.buffer) > 65536:
                    raise ValueError('Oversized reply')
        line, self.buffer = self.buffer.split(b'\n', 1)
        return json.loads(line)

    def rpc(self, op, **fields):
        self.counter += 1
        request = dict(id=self.counter, op=op, **fields)
        start = time.monotonic()
        self.process.stdin.write((json.dumps(request, allow_nan=False) + '\n').encode())
        reply = self.read(2 if op in ('inspect', 'stop') else .3)
        if reply.get('id') != self.counter:
            raise RuntimeError('Out of order acknowledgment')
        if 'error' in reply:
            raise RuntimeError(reply['error'])
        self.log.write(json.dumps(dict(request=request, reply=reply, monotonic=start,
                                       rtt=time.monotonic() - start)) + '\n')
        self.log.flush()
        return reply['result']

    def close(self):
        if self.process:
            with contextlib.suppress(OSError):
                self.process.stdin.close()
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.terminate()
                try:
                    self.process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait()
            self.process.stdout.close()
            self.process = None
        self.stderr.close()
        self.log.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', required=True)
    parser.add_argument('--leader-port', required=True)
    parser.add_argument('--follower-port', required=True)
    parser.add_argument('--leader-id', default='go2_leader')
    parser.add_argument('--follower-id', default='go2_follower')
    parser.add_argument('--ssh-control-path')
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    (BASE / 'logs').mkdir(parents=True, exist_ok=True)
    run = Path(tempfile.mkdtemp(prefix='so101-teleop-', dir=BASE / 'logs'))
    print('Логи:', run, flush=True)
    leader = SO101Leader(SO101LeaderConfig(port=args.leader_port, id=args.leader_id))
    follower = SO101Follower(SO101FollowerConfig(port='unused', id=args.follower_id))
    calibration = {c.id: asdict(c) for c in follower.calibration.values()}
    remote = None
    terminal = None
    armed = False
    synchronized = True

    def interrupt(*_):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupt)
    try:
        leader.bus.connect()
        if not leader.bus.is_calibrated:
            raise RuntimeError('Leader calibration mismatch')
        remote = Remote(args, calibration, run)
        sample = remote.rpc('inspect')
        current = follower.bus._normalize({int(k): v for k, v in sample['positions'].items()})
        target = leader.bus.sync_read('Present_Position')
        differences = {name: round(target[name] - current[motor.id], 2)
                       for name, motor in leader.bus.motors.items()}
        print('Leader − follower (градусы, gripper в %):', differences, flush=True)
        print('Follower model/calibration/mode: OK; motors:', sample['motors'], flush=True)
        if any(abs(v) > 30 for v in differences.values()):
            raise RuntimeError('Align arms: initial difference exceeds 30')
        if args.check:
            print('CHECK PASSED: read only, torque unchanged', flush=True)
            return
        print('Телеоп руки через Go2: 30 Гц, плавность 60°/с. q или Ctrl+C — конец следования; момент остаётся включённым.', flush=True)
        print('Поддерживайте follower при остановке. Ноги Go2 не управляются.', flush=True)
        # Leader is hand-guided, as in the existing local teleop.
        leader.bus.disable_torque()
        # Refresh after console output and local USB operations.
        sample = remote.rpc('inspect')
        # Recheck poses immediately before torque-on.
        current = follower.bus._normalize({int(k): v for k, v in sample['positions'].items()})
        target = leader.bus.sync_read('Present_Position')
        if any(abs(target[name] - current[motor.id]) > 30 for name, motor in leader.bus.motors.items()):
            raise RuntimeError('Poses changed; align arms before enabling torque')
        armed = True
        sample = remote.rpc('arm', sampled=sample['sampled'], accept_powered=True)
        if sys.stdin.isatty():
            terminal = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin)
        print('RUNNING — двигайте leader; q / Ctrl+C — остановка', flush=True)
        count = 0
        report = time.monotonic()
        while True:
            start = time.monotonic()
            if select.select([sys.stdin], [], [], 0)[0]:
                key = os.read(sys.stdin.fileno(), 1)
                if key in (b'q', b'Q', b'\x1b', b''):
                    break
            target = leader.bus.sync_read('Present_Position')
            raw = follower.bus._unnormalize({motor.id: target[name] for name, motor in follower.bus.motors.items()})
            try:
                sample = remote.rpc('action', positions=raw, sampled=sample['sampled'])
            except BaseException:
                synchronized = False
                raise
            count += 1
            elapsed = time.monotonic() - report
            if elapsed >= 5:
                print('Телеоп %.1f Гц, связь OK' % (count / elapsed), flush=True)
                count = 0
                report = time.monotonic()
            time.sleep(max(0, start + 1 / 30 - time.monotonic()))
    except KeyboardInterrupt:
        print('\nОстановка...', flush=True)
    finally:
        if terminal is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, terminal)
        try:
            if remote:
                try:
                    if armed and synchronized:
                        remote.rpc('stop')
                        print('Follower: удержание выключено, подтверждено чтением.', flush=True)
                finally:
                    remote.close()
        finally:
            if leader.bus.is_connected:
                leader.bus.disconnect(disable_torque=False)
            print('Сессия завершена. Логи:', run, flush=True)


if __name__ == '__main__':
    main()
