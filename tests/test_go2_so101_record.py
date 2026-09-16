import copy
import json
import sys
import time
from pathlib import Path

import numpy as np
import pytest
import yaml

BASE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BASE / 'scripts'))
from go2_so101_record import (  # noqa: E402
    Controller,
    Hardware,
    create_ui,
    features,
    record_session,
    validate_output,
)


def config(tmp_path):
    cfg = yaml.safe_load((BASE / 'configs/go2_dataset_collection.yaml').read_text())
    extra = yaml.safe_load((BASE / 'configs/go2_so101_collection.yaml').read_text())
    extra.pop('base_config')
    for key, value in extra.items():
        if isinstance(value, dict):
            cfg.setdefault(key, {}).update(value)
        else:
            cfg[key] = value
    cfg['dataset'].update(num_episodes=1, countdown_seconds=0, reset_seconds=.05,
                          episode_seconds=.35, repo_id='local/mock_combined')
    cfg['network']['ssh_control_path'] = str(tmp_path / 'ssh.sock')
    return cfg


def test_two_camera_dataset_actual_actions_resume(tmp_path):
    import pyarrow.parquet as pq
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    cfg = config(tmp_path)
    root = tmp_path / 'dataset'
    for attempt in range(2):
        run = tmp_path / str(attempt)
        run.mkdir()
        hw = Hardware(copy.deepcopy(cfg), run, mock=True)
        ui = create_ui(cfg, headless=True)
        assert record_session(cfg, root, hw, ui, resume=bool(attempt)) == 1
        assert all(hw.arm.arm.bus.torque.values())
    ds = LeRobotDataset('local/mock_combined', root=root, video_backend='pyav')
    assert ds.meta.total_episodes == 2 and ds.fps == 20
    for key in ('front', 'wrist'):
        assert ds[-1]['observation.images.' + key].shape == (3, 480, 640)
    assert ds[-1]['observation.state'].shape == (12,) and ds[-1]['action'].shape == (9,)
    recorded = []
    for attempt in range(2):
        run = tmp_path / str(attempt)
        commands = {row['observed_at']: row for row in map(json.loads, (run / 'combined.jsonl').read_text().splitlines())}
        for row in map(json.loads, (run / 'frames.jsonl').read_text().splitlines()):
            if 'observed_at' in row:
                np.testing.assert_array_equal(row['action'], commands[row['observed_at']]['action'])
                recorded.append(row['action'])
    stored = []
    for p in sorted((root / 'data').rglob('*.parquet')):
        stored.extend(pq.read_table(p, columns=['action'])['action'].to_pylist())
    np.testing.assert_array_equal(np.asarray(stored), np.asarray(recorded))


def test_root_and_schema_protection(tmp_path, monkeypatch):
    import go2_so101_record as recording
    monkeypatch.setattr(recording, 'BASE', tmp_path)
    cfg = config(tmp_path)
    with pytest.raises(ValueError):
        validate_output(cfg, tmp_path / 'datasets/go2_only/old', False, False)
    root = tmp_path / 'datasets/go2_so101/real'
    root.mkdir(parents=True)
    with pytest.raises(FileExistsError):
        validate_output(cfg, root, False, False)
    with pytest.raises(ValueError):
        validate_output(cfg, root, True, True)
    (root / 'meta').mkdir()
    (root / 'meta/info.json').write_text(json.dumps(dict(codebase_version='v3.0', robot_type='unitree_go2',
                                                        fps=20, features=features())))
    with pytest.raises(ValueError):
        validate_output(cfg, root, True, False)


class DummyHardware:
    def __init__(self):
        self.actions = []
        self.stopped = False

    def observe(self):
        return {}

    def step(self, obs, base, live):
        self.actions.append(live)
        return np.zeros(9)

    def close(self):
        self.stopped = True


def test_ui_loss_holds_and_recording_loss_aborts():
    hw = DummyHardware()
    controller = Controller(hw)
    controller.publish(True, dict.fromkeys(('base.vx', 'base.vy', 'base.wz'), .1))
    controller.thread.start()
    try:
        time.sleep(.3)
        assert True in hw.actions and hw.actions[-1] is False
        controller.publish(True, {}, episode=1)
        time.sleep(.35)
        with pytest.raises(RuntimeError, match='current episode'):
            controller.check()
    finally:
        controller.close()
    assert hw.stopped


def test_transport_failure_discards_partial_episode(tmp_path):
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    cfg = config(tmp_path)
    root = tmp_path / 'dataset'
    good = tmp_path / 'good'
    good.mkdir()
    record_session(cfg, root, Hardware(copy.deepcopy(cfg), good, mock=True), create_ui(cfg, True))
    before = json.loads((root / 'meta/info.json').read_text())
    run = tmp_path / 'bad'
    run.mkdir()
    hw = Hardware(copy.deepcopy(cfg), run, mock=True)
    original = hw.step
    calls = [0]
    def fail(obs, action, live):
        calls[0] += 1
        if calls[0] == 5:
            hw.arm_error = ConnectionError('Injected arm disconnect')
            raise ConnectionError('Injected arm disconnect')
        return original(obs, action, live)
    hw.step = fail
    cfg['dataset']['episode_seconds'] = 1
    with pytest.raises(RuntimeError, match='current episode'):
        record_session(cfg, root, hw, create_ui(cfg, True), resume=True)
    after = json.loads((root / 'meta/info.json').read_text())
    assert after['total_episodes'] == before['total_episodes'] == 1
    assert after['total_frames'] == before['total_frames']
    ds = LeRobotDataset('local/mock_combined', root=root, video_backend='pyav')
    assert len(ds) == before['total_frames']
    assert all(hw.arm.arm.bus.torque.values())


def test_arm_continues_during_camera_timeout(tmp_path):
    from go2_network import StaleNetworkFrame
    cfg = config(tmp_path)
    hw = Hardware(cfg, tmp_path, mock=True)
    hw.connect()
    hw.enable()
    controller = Controller(hw)
    original = hw.base.get_observation
    calls = [0]

    def delayed_camera():
        calls[0] += 1
        if calls[0] == 1:
            time.sleep(.35)
            raise StaleNetworkFrame('injected camera delay')
        return original()

    hw.base.get_observation = delayed_camera
    controller.publish(True, dict.fromkeys(('base.vx', 'base.vy', 'base.wz'), 0.), episode=4)
    first = hw.arm_snapshot()[1]['sampled']
    controller.thread.start()
    try:
        end = time.monotonic() + .8
        while time.monotonic() < end:
            controller.publish(True, dict.fromkeys(('base.vx', 'base.vy', 'base.wz'), 0.), episode=4)
            time.sleep(.02)
        controller.check()
        assert controller.network_fault
        assert controller.network_ready
        assert 4 in controller.invalid_episodes
        assert hw.arm_snapshot()[1]['sampled'] > first + .5
        assert hw.arm.arm.armed
        assert all(hw.arm.arm.bus.torque.values())
        assert controller.frames.empty()
    finally:
        controller.close()
    assert all(hw.arm.arm.bus.torque.values())
