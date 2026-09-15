#!/usr/bin/env python3
"""Hardware-free smoke test."""

from lerobot_robot_unitree_go2 import UnitreeGo2, UnitreeGo2Config

with UnitreeGo2(UnitreeGo2Config(mock=True)) as robot:
    print(robot.observation_features)
    print(robot.get_observation()["front"].shape)
    print(robot.send_action({"base.vx": 0.1, "base.vy": 0.0, "base.wz": 0.0}))
