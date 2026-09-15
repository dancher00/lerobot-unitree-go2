#!/usr/bin/env python3
"""Minimal gamepad/keyboard control loop with guaranteed cleanup."""

import argparse
import time

from lerobot.cameras.realsense import RealSenseCameraConfig

from lerobot_robot_unitree_go2 import UnitreeGo2, UnitreeGo2Config
from lerobot_robot_unitree_go2.teleoperators import (
    UnitreeGo2GamepadTeleop,
    UnitreeGo2GamepadTeleopConfig,
    UnitreeGo2KeyboardTeleop,
    UnitreeGo2KeyboardTeleopConfig,
)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface", default="eth0")
    parser.add_argument("--serial", required=True)
    parser.add_argument("--keyboard", action="store_true")
    args = parser.parse_args()
    config = UnitreeGo2Config(
        network_interface=args.interface,
        cameras={
            "front": RealSenseCameraConfig(
                serial_number_or_name=args.serial, width=640, height=480, fps=30
            )
        },
    )
    robot = UnitreeGo2(config)
    teleop = (
        UnitreeGo2KeyboardTeleop(UnitreeGo2KeyboardTeleopConfig())
        if args.keyboard
        else UnitreeGo2GamepadTeleop(UnitreeGo2GamepadTeleopConfig())
    )
    teleop.connect()
    robot.connect()
    period = 1.0 / config.control_frequency
    try:
        while True:
            started = time.monotonic()
            robot.send_action(teleop.get_action())
            delay = period - (time.monotonic() - started)
            if delay > 0:
                time.sleep(delay)
    except KeyboardInterrupt:
        pass
    finally:
        robot.emergency_stop("teleoperation ended")
        robot.disconnect()
        teleop.disconnect()


if __name__ == "__main__":
    main()
