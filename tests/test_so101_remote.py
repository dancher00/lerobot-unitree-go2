import importlib.util
import time
from pathlib import Path

import pytest

spec = importlib.util.spec_from_file_location('so101_endpoint', Path(__file__).resolve().parents[1] / 'scripts/so101_remote_endpoint.py')
endpoint = importlib.util.module_from_spec(spec)
spec.loader.exec_module(endpoint)


class FakeBus:
    def __init__(self):
        self.torque = dict.fromkeys(range(1, 7), 0)
        self.writes = []

    def read(self, mid, addr, count):
        return self.torque[mid] if addr == 40 else 2048

    def sync(self, addr, count, values):
        self.writes.append((addr, dict(values)))
        if addr == 40:
            self.torque.update(values)

    def close(self):
        pass


def make_arm(timeout=.5):
    bus = FakeBus()
    arm = endpoint.Arm(bus, {i: dict(range_min=0, range_max=4095) for i in range(1, 7)}, timeout)
    return arm, bus


def test_readonly_close_never_writes():
    arm, bus = make_arm()
    arm.sample()
    arm.close()
    assert bus.writes == []


def test_seed_rate_limit_stop_and_watchdog():
    arm, bus = make_arm(.06)
    try:
        sample = arm.sample()
        sample = arm.handle(dict(op='arm', sampled=sample['sampled']))
        assert bus.writes[0] == (42, dict.fromkeys(range(1, 7), 2048))
        assert all(bus.torque.values())
        time.sleep(.02)
        result = arm.handle(dict(op='action', sampled=sample['sampled'], positions=dict.fromkeys(range(1, 7), 4095)))
        assert 2048 <= result['sent'][1] < 2117
        time.sleep(.12)
        assert all(bus.torque.values())
        sample = arm.sample()
        with pytest.raises(RuntimeError, match='latched'):
            arm.handle(dict(op='action', sampled=sample['sampled'], positions=dict.fromkeys(range(1, 7), 2048)))
    finally:
        arm.close()


def test_stale_arm_and_nonfinite_rejected():
    arm, bus = make_arm()
    try:
        sample = arm.sample()
        with pytest.raises(TimeoutError):
            arm.handle(dict(op='arm', sampled=sample['sampled'] - 1))
        assert not bus.writes
        sample = arm.handle(dict(op='arm', sampled=sample['sampled']))
        with pytest.raises(ValueError):
            arm.handle(dict(op='action', sampled=sample['sampled'], positions=dict.fromkeys(range(1, 7), float('nan'))))
    finally:
        arm.close()
    assert all(bus.torque.values())
    assert not any(addr == 40 and any(v == 0 for v in values.values()) for addr, values in bus.writes)


def test_read_packet_matches_feetech_protocol():
    assert endpoint.packet(1, 2, [56, 2]) == bytes.fromhex('ffff0104023802be')


def test_stop_close_keep_last_goal_and_torque():
    arm, bus = make_arm()
    sample = arm.sample()
    arm.handle(dict(op='arm', sampled=sample['sampled']))
    before = list(bus.writes)
    arm.stop()
    arm.close()
    assert bus.writes == before
    assert all(bus.torque.values())
    assert not arm.armed


def test_powered_arm_requires_explicit_takeover():
    arm, bus = make_arm()
    try:
        bus.torque = dict.fromkeys(range(1, 7), 1)
        sample = arm.sample()
        with pytest.raises(RuntimeError, match='takeover'):
            arm.handle(dict(op='arm', sampled=sample['sampled']))
        assert not bus.writes
        arm.handle(dict(op='arm', sampled=sample['sampled'], accept_powered=True))
        assert bus.writes[0][0] == 42
        assert arm.armed
    finally:
        arm.close()
