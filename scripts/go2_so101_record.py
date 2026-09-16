"""Two-camera, joint base/arm teleoperation recording in LeRobot Dataset v3."""
import argparse
import fcntl
import json
import math
import os
import queue
import signal
import sys
import tempfile
import threading
import time
import traceback
from concurrent.futures import ThreadPoolExecutor
from dataclasses import asdict
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import yaml
from go2_network import make_network_robot
from so101_remote_endpoint import Arm
from teleoperate_so101_go2 import Remote

BASE = Path(__file__).resolve().parents[1]
JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']
BASE_STATE = ['base.vx', 'base.vy', 'base.wz', 'base.roll', 'base.pitch', 'base.yaw']
STATE = BASE_STATE + ['arm.' + name + '.pos' for name in JOINTS]
ACTION = BASE_STATE[:3] + ['arm.' + name + '.pos' for name in JOINTS]
ROBOT_TYPE = 'go2_so101_combined'


def features():
    return {
        'observation.state': dict(dtype='float32', shape=(12,), names=STATE),
        'action': dict(dtype='float32', shape=(9,), names=ACTION),
        **{'observation.images.' + cam: dict(dtype='video', shape=(480, 640, 3),
                                            names=['height', 'width', 'channels']) for cam in ('front', 'wrist')},
    }


def endpoint_source():
    # Both modules are sent in memory. The onboard Python needs no new packages/files.
    original = (BASE / 'scripts/go2_network_server.py').read_text()
    extension = (BASE / 'scripts/go2_dual_camera_endpoint.py').read_text()
    return ("import types,sys,json,argparse\n"
            "m=types.ModuleType('go2_stream');sys.modules['go2_stream']=m\n"
            f"exec(compile({original!r},'<go2_stream>','exec'),m.__dict__)\n"
            f"exec(compile({extension!r},'<dual_camera>','exec'))\n"
            "p=argparse.ArgumentParser();p.add_argument('--config',required=True);"
            "p.add_argument('--mock',action='store_true');p.add_argument('--readonly',action='store_true')\n"
            "a=p.parse_args();m.serve(json.loads(a.config),a.mock,a.readonly)\n").encode()


class MockBus:
    def __init__(self, calibration):
        self.cal = calibration
        self.pos = {i: int((c['range_min'] + c['range_max']) / 2) for i, c in calibration.items()}
        self.torque = dict.fromkeys(calibration, 0)

    def read(self, mid, address, count):
        c = self.cal[mid]
        off = c['homing_offset']
        return {3: 777, 9: c['range_min'], 11: c['range_max'],
                31: (abs(off) | 2048) if off < 0 else off,
                33: 0, 40: self.torque[mid], 56: self.pos[mid], 21: 16, 23: 0, 22: 32}[address]

    def sync(self, address, count, values):
        if address == 40:
            self.torque.update(values)
        elif address == 42:
            self.pos.update(values)

    def close(self):
        pass


class MockRemote:
    def __init__(self, calibration):
        self.arm = Arm(MockBus(calibration), calibration)

    def rpc(self, op, **fields):
        return self.arm.handle(dict(op=op, **fields))

    def close(self):
        self.arm.close()


class Hardware:
    def __init__(self, cfg, run, mock=False, readonly=False):
        from lerobot.motors import MotorCalibration
        from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
        from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig
        self.cfg, self.run, self.mock, self.readonly = cfg, run, mock, readonly
        self.pool = ThreadPoolExecutor(max_workers=3)
        self.log = (run / 'combined.jsonl').open('x')
        self.base_log = (run / 'base.jsonl').open('x')
        self.base = make_network_robot(cfg, mock=mock, readonly=readonly, log=self.base_log,
                                      endpoint_stderr_path=run / 'base.stderr.log', endpoint_source=endpoint_source())
        self.follower = SO101Follower(SO101FollowerConfig(port='unused', id=cfg['arm']['follower_id']))
        if mock:
            self.follower.calibration = {n: MotorCalibration(id=i, drive_mode=0, homing_offset=0,
                                                            range_min=500, range_max=3500)
                                         for i, n in enumerate(JOINTS, 1)}
            self.follower.bus.calibration = self.follower.calibration
        self.leader = None if mock else SO101Leader(SO101LeaderConfig(port=cfg['arm']['leader_port'],
                                                                     id=cfg['arm']['leader_id']))
        self.arm = None
        self.arm_broken = False
        self.enabled = False
        self.arm_closed = threading.Event()
        self.arm_thread = None
        self.arm_latest = None
        self.arm_error = None
        self.arm_intent = (False, time.monotonic())
        self.arm_applied_at = 0.
        self.arm_lock = threading.Lock()

    def connect(self):
        if self.leader:
            self.leader.bus.connect()
            if not self.leader.bus.is_calibrated:
                raise RuntimeError('Leader calibration mismatch')
        cal = {v.id: asdict(v) for v in self.follower.calibration.values()}
        args = SimpleNamespace(host=self.cfg['network']['host'], follower_port=self.cfg['arm']['follower_port'],
                               ssh_control_path=self.cfg['network']['ssh_control_path'])
        self.arm = MockRemote(cal) if self.mock else Remote(args, cal, self.run)
        sample = self.arm.rpc('inspect')
        if not self.readonly:
            self.check_alignment(sample)
        self.base.connect()

    def arm_rpc(self, op, **fields):
        try:
            return self.arm.rpc(op, **fields)
        except BaseException:
            self.arm_broken = True
            raise

    def leader_goal(self):
        if self.mock:
            # Deterministic finite synthetic intent for testing the complete recorder.
            x = math.sin(time.monotonic() * 2) * 8
            return dict(zip(JOINTS, [x, x / 2, -x, x, x, 50 + x], strict=True))
        return self.leader.bus.sync_read('Present_Position')

    def normalize(self, positions):
        values = self.follower.bus._normalize({int(k): v for k, v in positions.items()})
        return np.asarray([values[i] for i in range(1, 7)], dtype=np.float32)

    def check_alignment(self, sample):
        delta = np.asarray(list(self.leader_goal().values())) - self.normalize(sample['positions'])
        print('Leader − follower:', dict(zip(JOINTS, delta.round(2).tolist(), strict=True)), flush=True)
        if np.max(np.abs(delta)) > self.cfg['arm']['max_initial_difference']:
            raise RuntimeError('Align leader/follower: difference exceeds configured limit')

    def enable(self):
        if self.readonly:
            raise RuntimeError('Read-only mode cannot enable torque')
        sample = self.arm_rpc('inspect')
        self.check_alignment(sample)
        if self.leader:
            self.leader.bus.disable_torque()
        # Explicitly launched new session, calibration and pose checked above.
        self.arm_rpc('arm', sampled=sample['sampled'], accept_powered=True)
        self.enabled = True
        self.arm_thread = threading.Thread(target=self.arm_loop, name='independent-arm', daemon=True)
        self.arm_thread.start()
        deadline = time.monotonic() + 1
        while self.arm_latest is None:
            if self.arm_error:
                raise self.arm_error
            if time.monotonic() > deadline:
                raise TimeoutError('No independent arm sample')
            time.sleep(.005)

    def arm_loop(self):
        try:
            while not self.arm_closed.is_set():
                start = time.monotonic()
                live, updated = self.arm_intent
                sample = self.arm_rpc('sample')
                fields = dict(sampled=sample['sampled'])
                live = live and start - updated < .2
                if live:
                    goal = self.leader_goal()
                    fields['positions'] = self.follower.bus._unnormalize({i: goal[n] for i, n in enumerate(JOINTS, 1)})
                sent = self.arm_rpc('action' if live else 'hold', **fields)
                with self.arm_lock:
                    self.arm_latest = (sample, sent, time.monotonic())
                    self.arm_applied_at = updated
                self.arm_closed.wait(max(0, start + .05 - time.monotonic()))
        except BaseException as exc:
            self.arm_error = exc

    def arm_snapshot(self):
        if self.arm_error:
            raise self.arm_error
        with self.arm_lock:
            snapshot = self.arm_latest
        if snapshot is None or time.monotonic() - snapshot[2] > .2:
            raise TimeoutError('Independent arm state stale')
        return snapshot

    def observe(self):
        start = time.monotonic()
        futures = [self.pool.submit(self.base.get_observation),
                   self.pool.submit(lambda: self.arm_snapshot()[0]) if self.enabled else
                   self.pool.submit(self.arm_rpc, 'sample')]
        # Wait for both even if one failed before stopping shared hardware.
        results, errors = [], []
        for f in futures:
            try:
                results.append(f.result())
            except BaseException as exc:
                errors.append(exc)
        if errors:
            raise errors[0]
        base, arm = results
        packet = self.base.backend.sample
        if abs(packet['sampled_at'] - arm['sampled']) > .1:
            raise TimeoutError('Base/arm observation skew exceeds 100 ms')
        if time.monotonic() - start > .2:
            raise TimeoutError('Combined observation exceeded 200 ms')
        state = np.asarray([base[k] for k in BASE_STATE] + self.normalize(arm['positions']).tolist(), dtype=np.float32)
        if not np.isfinite(state).all():
            raise ValueError('Non-finite state')
        return dict(front=base['front'], wrist=base['wrist'], state=state,
                    arm=arm, base=packet, observed_at=start)

    def step(self, obs, base_action, live):
        if self.readonly or not self.enabled:
            raise RuntimeError('Motion not enabled')
        if not live:
            base_action = dict.fromkeys(BASE_STATE[:3], 0.)
        start = time.monotonic()
        base_sent = self.base.send_action(dict(base_action))
        _, arm_sent, _ = self.arm_snapshot()
        sent = np.asarray([base_sent[k] for k in BASE_STATE[:3]] + self.normalize(arm_sent['sent']).tolist(), dtype=np.float32)
        base_time = self.base.backend.last_sent['sent_monotonic']
        # Arm sample follows its USB write; a conservative bound for command skew.
        if abs(base_time - arm_sent['sampled']) > .1:
            raise TimeoutError('Base/arm command skew exceeds 100 ms')
        self.log.write(json.dumps(dict(observed_at=obs['observed_at'], command_start=start,
                                       command_done=time.monotonic(), state=obs['state'].tolist(),
                                       action=sent.tolist(), arm_raw=arm_sent['sent'],
                                       arm_sampled=obs['arm']['sampled'], base_sampled=obs['base']['sampled_at'],
                                       base_sent=base_time, arm_after_send=arm_sent['sampled'],
                                       cameras={k: {a: b for a, b in obs['base'][s].items() if a != 'jpeg'}
                                                for k, s in [('front', 'camera'), ('wrist', 'wrist')]})) + '\n')
        self.log.flush()
        return sent

    def close(self):
        self.arm_closed.set()
        if self.arm_thread:
            self.arm_thread.join(timeout=3)
            if self.arm_thread.is_alive():
                raise RuntimeError('Arm worker still active; refusing concurrent USB access')
        errors = []
        # Dispatch both stops without waiting for either link's round trip first.
        def stop_arm():
            if self.arm:
                try:
                    if self.enabled and not self.arm_broken:
                        self.arm.rpc('stop')
                finally:
                    self.arm.close()
        futures = [self.pool.submit(self.base.disconnect), self.pool.submit(stop_arm)]
        for f in futures:
            try:
                f.result()
            except BaseException as exc:
                errors.append(str(exc))
        self.pool.shutdown(wait=True)
        if self.leader and self.leader.bus.is_connected:
            self.leader.bus.disconnect(disable_torque=False)
        self.log.close()
        self.base_log.close()
        if errors:
            print('STOP/CLEANUP ERRORS:', errors, file=sys.stderr, flush=True)


class Controller:
    """Hardware thread remains alive while main/UI thread finalizes videos."""
    def __init__(self, hardware, fps=20):
        self.hardware, self.fps = hardware, fps
        self.condition = threading.Condition()
        self.closed = threading.Event()
        self.error = None
        self.latest = None
        self.network_fault = None
        self.network_ready = False
        self.invalid_episodes = set()
        self.frames = queue.Queue(maxsize=40)
        self.command = (False, dict.fromkeys(BASE_STATE[:3], 0.), None, time.monotonic(), 0)
        self.processed_revision = -1
        self.thread = threading.Thread(target=self.run, name='combined-control', daemon=True)

    def publish(self, live, base_action, episode=None):
        if hasattr(self.hardware, 'arm_intent'):
            self.hardware.arm_intent = (live, time.monotonic())
        with self.condition:
            revision = self.command[-1] + 1
            self.command = live, dict(base_action), episode, time.monotonic(), revision
            return revision

    def check(self):
        if self.error:
            raise RuntimeError('Combined control stopped; current episode must be discarded') from self.error

    def pause(self):
        rev = self.publish(False, dict.fromkeys(BASE_STATE[:3], 0.))
        if hasattr(self.hardware, 'arm_applied_at'):
            updated = self.hardware.arm_intent[1]
            deadline = time.monotonic() + .5
            while self.hardware.arm_applied_at < updated:
                if self.hardware.arm_error:
                    raise self.hardware.arm_error
                if time.monotonic() >= deadline:
                    raise TimeoutError('Independent arm hold not acknowledged')
                time.sleep(.005)
        with self.condition:
            if not self.condition.wait_for(lambda: self.processed_revision >= rev or self.error, timeout=2):
                raise TimeoutError('Could not confirm base zero / arm hold')
        self.check()

    def run(self):
        try:
            while not self.closed.is_set():
                start = time.monotonic()
                with self.condition:
                    live, base_action, episode, updated, rev = self.command
                if time.monotonic() - updated > .2:
                    live = False
                    if episode is not None:
                        raise TimeoutError('Recording UI unresponsive for 200 ms')
                try:
                    obs = self.hardware.observe()
                    if self.network_fault:
                        self.latest = obs
                        self.network_ready = True
                        with self.condition:
                            self.processed_revision = rev
                            self.condition.notify_all()
                        self.closed.wait(.05)
                        continue
                    sent = self.hardware.step(obs, base_action, live)
                except (TimeoutError, ConnectionError) as exc:
                    if not hasattr(self.hardware, 'arm_error') or self.hardware.arm_error:
                        raise
                    if episode is not None:
                        self.invalid_episodes.add(episode)
                    if self.network_fault is None:
                        print('Go2/VIDEO PAUSED; arm remains independent. n after recovery, q exit. Reason:', exc, flush=True)
                        try:
                            self.hardware.base.emergency_stop('network delay')
                        except Exception as stop_error:
                            print('Go2 STOP NOT CONFIRMED:', stop_error, flush=True)
                    self.network_fault = str(exc)
                    self.network_ready = False
                    with self.condition:
                        self.processed_revision = rev
                        self.condition.notify_all()
                    self.closed.wait(.1)
                    continue
                if episode is not None:
                    self.frames.put_nowait((episode, obs, sent))
                with self.condition:
                    self.latest = obs
                    self.processed_revision = rev
                    self.condition.notify_all()
                self.closed.wait(max(0, start + 1 / self.fps - time.monotonic()))
        except BaseException as exc:
            with self.condition:
                self.error = exc
                self.condition.notify_all()
        finally:
            self.hardware.close()

    def close(self):
        self.closed.set()
        self.thread.join(timeout=8)
        if self.thread.is_alive():
            raise RuntimeError('Hardware thread has not stopped; check robot and arm power')


def create_ui(cfg, headless=False):
    from go2_recording_ui import RecordingUI, pygame

    class DualUI(RecordingUI):
        def connect(self, calibrate=True):
            super().connect(calibrate)
            self.screen = pygame.display.set_mode((1280, 576))
            pygame.display.set_caption('Go2 + SO-101 | front / wrist | n next, r retry, q finish')

        def display(self, obs):
            super().display(obs)
            wrist = pygame.image.frombuffer(obs['wrist'].tobytes(), (640, 480), 'RGB')
            self.screen.blit(wrist, (640, 0))
            self.screen.fill((20, 20, 20), (640, 480, 640, 96))
            label = 'ARM: leader | SPACE/k: hold both | x lock, v unlock'
            self.screen.blit(self.font.render(label, True, (240, 240, 240)), (650, 512))
            pygame.display.flip()

    if not headless:
        return DualUI(cfg)

    class Headless:
        events = None
        phase = ''

        def __init__(self):
            self.events = dict(exit_early=False, rerecord_episode=False, stop_recording=False)
            self.ready = threading.Event()
            self.ready.set()

        def connect(self):
            pass

        def display(self, obs):
            pass

        def get_action(self):
            return dict(zip(BASE_STATE[:3], [.1, 0., .1], strict=True))

        def block_reason(self):
            return None

        def reset_controls(self):
            pass

        def disconnect(self):
            pass

    return Headless()


def validate_output(cfg, root, resume, mock):
    allowed = (BASE / 'datasets/go2_so101').resolve()
    if not root.is_relative_to(allowed) or root == allowed:
        raise ValueError('Output must be a dataset inside datasets/go2_so101')
    if mock and (not root.name.startswith('mock_') or not cfg['dataset']['repo_id'].split('/')[-1].startswith('mock_')):
        raise ValueError('Mock requires separate mock_ dataset name/path')
    if not mock and root.name.startswith('mock_'):
        raise ValueError('Real data cannot be appended to a mock dataset')
    if root.exists() and not resume:
        raise FileExistsError('Dataset exists; use --resume explicitly: ' + str(root))
    if resume:
        info = json.loads((root / 'meta/info.json').read_text())
        if info['codebase_version'] != 'v3.0' or info['robot_type'] != ROBOT_TYPE or info['fps'] != 20:
            raise ValueError('Dataset type/version/fps mismatch')
        for key, expected in features().items():
            actual = info['features'][key]
            if (actual['dtype'] != expected['dtype'] or list(actual['shape']) != list(expected['shape'])
                    or actual['names'] != expected['names']):
                raise ValueError('Dataset feature mismatch: ' + key)


def record_session(cfg, root, hardware, ui, resume=False):
    from lerobot.configs.video import RGBEncoderConfig
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    dataset = None
    controller = None
    pending_frames = 0
    frame_log = (hardware.run / 'frames.jsonl').open('x')
    try:
        hardware.connect()
        # Create/load dataset before enabling follower torque.
        opts = dict(repo_id=cfg['dataset']['repo_id'], root=root, streaming_encoding=True,
                    encoder_threads=2, rgb_encoder=RGBEncoderConfig(vcodec='h264', preset='veryfast'),
                    video_backend='pyav')
        dataset = (LeRobotDataset.resume(**opts) if resume else
                   LeRobotDataset.create(fps=20, features=features(), robot_type=ROBOT_TYPE, **opts))
        ui.connect()
        hardware.enable()
        controller = Controller(hardware)
        controller.thread.start()
        token = 0
        recorded = 0

        def update(phase, episode=None, live=True):
            controller.check()
            ui.phase = ('GO2/VIDEO PAUSED | arm live | n resume, q quit'
                        if controller.network_fault else phase)
            if controller.latest is not None:
                ui.display(controller.latest)
            action = ui.get_action()
            if controller.network_fault and controller.network_ready and ui.events['exit_early'] and episode is None:
                controller.network_fault = None
                controller.network_ready = False
                ui.events.update(exit_early=False, rerecord_episode=False)
                ui.reset_controls()
                action = dict.fromkeys(BASE_STATE[:3], 0.)
            if episode in controller.invalid_episodes:
                ui.events.update(exit_early=True, rerecord_episode=True)
            allow_arm = ui.block_reason() in (None, 'no_keys')
            controller.publish(live and allow_arm and not ui.events['stop_recording'], action, episode)

        def drain(episode):
            nonlocal pending_frames
            count = 0
            while True:
                try:
                    frame_id, obs, sent = controller.frames.get_nowait()
                except queue.Empty:
                    return count
                if frame_id != episode:
                    raise RuntimeError('Episode boundary mismatch')
                dataset.add_frame({'observation.state': obs['state'], 'action': sent,
                                   'observation.images.front': obs['front'],
                                   'observation.images.wrist': obs['wrist'], 'task': cfg['task']})
                pending_frames += 1
                frame_log.write(json.dumps(dict(attempt=episode, episode_index=dataset.meta.total_episodes,
                                                frame_index=pending_frames-1,
                                                observed_at=obs['observed_at'], action=sent.tolist())) + '\n')
                frame_log.flush()
                count += 1

        print('ПРЕВЬЮ: arm — leader; Go2 — i/, j/l, Shift+j/l. Enter — запись; q — выход.', flush=True)
        while not ui.ready.is_set() and not ui.events['stop_recording']:
            update('PREVIEW | arm: leader | Enter record, q quit')
            time.sleep(.02)
        controller.pause()
        while recorded < cfg['dataset']['num_episodes'] and not ui.events['stop_recording']:
            while controller.network_fault and not ui.events['stop_recording']:
                update('WAIT FOR GO2/VIDEO | n resume')
                time.sleep(.02)
            if ui.events['stop_recording']:
                break
            ui.events.update(exit_early=False, rerecord_episode=False)
            ui.reset_controls()
            end = time.monotonic() + cfg['dataset']['countdown_seconds']
            previous = None
            while time.monotonic() < end and not ui.events['stop_recording']:
                left = math.ceil(end - time.monotonic())
                if left != previous:
                    print(f'Эпизод {recorded+1}/{cfg["dataset"]["num_episodes"]}: запись через {left}...', flush=True)
                    previous = left
                update(f'COUNTDOWN {left}s | n ready, q quit')
                if ui.events['exit_early']:
                    break
                time.sleep(.02)
            if ui.events['stop_recording']:
                break
            ui.events.update(exit_early=False, rerecord_episode=False)
            ui.reset_controls()
            token += 1
            frames = 0
            start = time.monotonic()
            print(f'ЗАПИСЬ {recorded+1}/{cfg["dataset"]["num_episodes"]}: n — сохранить, r — повторить, q — сохранить и выйти', flush=True)
            while time.monotonic() - start < cfg['dataset']['episode_seconds']:
                elapsed = time.monotonic() - start
                update(f'RECORD {recorded+1}/{cfg["dataset"]["num_episodes"]} {elapsed:.1f}s | n/r/q', token)
                frames += drain(token)
                if ui.events['exit_early'] or ui.events['stop_recording']:
                    break
                time.sleep(.01)
            # Wait for base zero + fixed arm pose BEFORE saving/encoding.
            controller.pause()
            frames += drain(token)
            if ui.events['rerecord_episode'] or token in controller.invalid_episodes or frames == 0:
                dataset.clear_episode_buffer()
                pending_frames = 0
                frame_log.write(json.dumps(dict(attempt=token, discarded=True)) + '\n')
                print('Эпизод отброшен; повтор.', flush=True)
            else:
                dataset.save_episode()
                pending_frames = 0
                frame_log.write(json.dumps(dict(attempt=token, saved_episode=dataset.meta.total_episodes-1)) + '\n')
                frame_log.flush()
                recorded += 1
                print(f'Сохранён эпизод {dataset.meta.total_episodes-1}: {frames} кадров, {frames/20:.2f} с', flush=True)
            if recorded >= cfg['dataset']['num_episodes'] or ui.events['stop_recording']:
                break
            ui.events.update(exit_early=False, rerecord_episode=False)
            ui.reset_controls()
            end = time.monotonic() + cfg['dataset']['reset_seconds']
            print('ПОДГОТОВКА: телеоп включён; n — готов, q — выход.', flush=True)
            while time.monotonic() < end and not ui.events['stop_recording']:
                update(f'RESET {math.ceil(end-time.monotonic())}s | n ready, q quit')
                if ui.events['exit_early']:
                    break
                time.sleep(.02)
            controller.pause()
        controller.pause()
        return recorded
    except BaseException:
        # Never save a partial episode after a transport/camera/motor failure or Ctrl+C.
        if controller:
            controller.close()
            controller = None
            hardware = None
        if dataset is not None and pending_frames:
            dataset.clear_episode_buffer()
        raise
    finally:
        try:
            if controller:
                controller.close()
            elif hardware:
                hardware.close()
        finally:
            ui.disconnect()
            if dataset:
                dataset.finalize()
            frame_log.close()


def transport_failure(exc):
    """Only transport/freshness faults are retryable, not calibration or arbitrary bugs."""
    seen = set()
    while exc is not None and id(exc) not in seen:
        seen.add(id(exc))
        if isinstance(exc, (TimeoutError, ConnectionError)):
            return True
        exc = exc.__cause__
    return False


def record_with_recovery(cfg, root, run, resume=False, mock=False, headless=False,
                         confirm=None):
    """Keep the launcher alive, but require explicit consent before a new armed session.

    record_session fully stops/closes hardware and discards its partial episode before
    returning an error. Never reuse a timed-out protocol or rearm a latched watchdog.
    """
    def total():
        info = root / 'meta/info.json'
        return json.loads(info.read_text())['total_episodes'] if info.exists() else 0

    initial = total()
    target = cfg['dataset']['num_episodes']
    while True:
        saved = total() - initial
        if saved >= target:
            return saved
        cfg['dataset']['num_episodes'] = target - saved
        hardware = Hardware(cfg, run, mock)
        ui = create_ui(cfg, headless)
        try:
            return saved + record_session(cfg, root, hardware, ui, resume)
        except Exception as exc:
            if mock or headless or not transport_failure(exc):
                raise
            (run / 'failure.txt').write_text(traceback.format_exc())
            print('\nПАУЗА: задержка/потеря связи. Незавершённая попытка отброшена.\n'
                  'Сохранённые эпизоды остаются. Момент руки программно не отключается.\n'
                  'Если остановка Go2 не подтверждена, проверьте её штатным пультом.\n'
                  'Убедитесь, что обе платформы остановлены и позы рук согласованы.\n'
                  f'Причина: {exc.__cause__ or exc}\nЛог: {run / "failure.txt"}', flush=True)
            prompt = ('Введите reconnect для нового подключения и включения удержания руки; '
                      'q — завершить: ')
            ask = confirm if confirm is not None else input
            while True:
                try:
                    answer = ask(prompt).strip().lower()
                except (EOFError, KeyboardInterrupt):
                    return total() - initial
                if answer == 'q':
                    return total() - initial
                if answer == 'reconnect':
                    break
            saved = total() - initial
            if saved >= target:
                return saved
            # New logs, new SSH channels, new endpoint watchdogs. No automatic takeover.
            parent = run.parent
            run = Path(tempfile.mkdtemp(prefix='go2-so101-retry-', dir=parent))
            cfg['network']['ssh_control_path'] = str(run / 'ssh.sock')
            (run / 'session.json').write_text(json.dumps(dict(config=cfg, root=str(root),
                                                             recovery=True), indent=2))
            resume = (root / 'meta/info.json').exists()
            validate_output(cfg, root, resume, mock)
            print('Переподключение после подтверждения. Логи:', run, flush=True)


def main():
    import av
    from datasets.utils.logging import disable_progress_bar
    av.logging.set_level(av.logging.ERROR)
    disable_progress_bar()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=str(BASE / 'configs/go2_so101_collection.yaml'))
    parser.add_argument('--resume', action='store_true')
    parser.add_argument('--episodes', type=int)
    parser.add_argument('--episode-seconds', type=float)
    parser.add_argument('--mock', action='store_true')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--readonly-check', action='store_true')
    parser.add_argument('--check-seconds', type=float, default=10)
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()
    extra = yaml.safe_load(Path(args.config).read_text())
    cfg = yaml.safe_load((BASE / extra.pop('base_config')).read_text())
    for key, value in extra.items():
        if isinstance(value, dict):
            cfg.setdefault(key, {}).update(value)
        else:
            cfg[key] = value
    for key, value in [('num_episodes', args.episodes), ('episode_seconds', args.episode_seconds)]:
        if value is not None:
            cfg['dataset'][key] = value
    cfg['dataset']['repo_id'] = os.environ.get('DATASET_REPO_ID', cfg['dataset']['repo_id'])
    cfg['task'] = os.environ.get('TASK', cfg['task'])
    cfg['network']['host'] = os.environ.get('GO2_HOST', cfg['network']['host'])
    cfg['network']['interface'] = os.environ.get('NETWORK_INTERFACE', cfg['network']['interface'])
    cfg['camera']['serial'] = os.environ.get('CAMERA_SERIAL', cfg['camera']['serial'])
    cfg['network']['wrist_device'] = os.environ.get('WRIST_CAMERA', cfg['network']['wrist_device'])
    cfg['arm']['leader_port'] = os.environ.get('LEADER_PORT', cfg['arm']['leader_port'])
    cfg['arm']['follower_port'] = os.environ.get('FOLLOWER_PORT', cfg['arm']['follower_port'])
    if cfg['control_frequency'] != 20 or cfg['dataset']['num_episodes'] < 1 or cfg['dataset']['episode_seconds'] <= 0:
        raise ValueError('Requires 20 Hz and positive episode count/duration')
    if args.headless and not args.mock and not args.readonly_check:
        raise ValueError('Real motion requires interactive camera window')
    root = (BASE / os.environ.get('DATASET_ROOT', cfg['dataset']['root'])).resolve()
    if args.mock:
        if 'DATASET_ROOT' not in os.environ:
            root = root.parent / ('mock_' + root.name + '_' + time.strftime('%Y%m%d_%H%M%S') + '_' + str(os.getpid()))
        if 'DATASET_REPO_ID' not in os.environ:
            cfg['dataset']['repo_id'] = 'local/' + root.name
        cfg['dataset'].update(countdown_seconds=0, reset_seconds=.1)
    if args.dry_run:
        print(json.dumps(dict(root=str(root), config=cfg, features=features()), indent=2))
        return
    if not args.readonly_check:
        validate_output(cfg, root, args.resume, args.mock)
    (BASE / 'logs').mkdir(parents=True, exist_ok=True)
    run = Path(tempfile.mkdtemp(prefix='go2-so101-', dir=BASE / 'logs'))
    cfg['network']['ssh_control_path'] = str(run / 'ssh.sock')
    (run / 'session.json').write_text(json.dumps(dict(config=cfg, options=vars(args), root=str(root)), indent=2))
    print('Логи:', run, '\nДатасет:', root, flush=True)
    def interrupt(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupt)
    if args.readonly_check:
        hardware = Hardware(cfg, run, args.mock, readonly=True)
        try:
            hardware.connect()
            count = 0
            start = time.monotonic()
            while time.monotonic() - start < args.check_seconds:
                tick = time.monotonic()
                obs = hardware.observe()
                count += 1
                time.sleep(max(0, tick + .05 - time.monotonic()))
            print('READONLY PASSED:', count, 'frames;', round(count/(time.monotonic()-start), 2),
                  'Hz; state', obs['state'].shape, 'front', obs['front'].shape, 'wrist', obs['wrist'].shape)
        finally:
            hardware.close()
        return
    root.parent.mkdir(parents=True, exist_ok=True)
    with (root.parent / ('.' + root.name + '.lock')).open('a') as lock:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        validate_output(cfg, root, args.resume, args.mock)
        print('При запуске включится удержание SO-101. Go2 должен быть подготовлен оператором; скрипт не поднимает его.', flush=True)
        recorded = record_with_recovery(cfg, root, run, args.resume, args.mock, args.headless)
        print('Готово. Сохранено новых эпизодов:', recorded, '\nДатасет:', root, flush=True)


if __name__ == '__main__':
    main()
