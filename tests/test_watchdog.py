import threading

from lerobot_robot_unitree_go2.safety import CommandWatchdog


def test_watchdog_timeout_stops_once() -> None:
    stopped = threading.Event()
    calls = 0

    def stop() -> None:
        nonlocal calls
        calls += 1
        stopped.set()

    watchdog = CommandWatchdog(0.05, stop)
    watchdog.start()
    watchdog.feed()
    assert stopped.wait(0.5)
    assert watchdog.tripped
    assert calls == 1
    watchdog.close()


def test_watchdog_disarm_prevents_stop() -> None:
    stopped = threading.Event()
    watchdog = CommandWatchdog(0.05, stopped.set)
    watchdog.start()
    watchdog.feed()
    watchdog.disarm()
    assert not stopped.wait(0.15)
    watchdog.close()
