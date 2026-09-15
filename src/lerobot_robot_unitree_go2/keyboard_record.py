"""Short command for recording a Go2 dataset with keyboard teleoperation."""

from __future__ import annotations

import argparse
import json
import sys
from collections.abc import Sequence

from .cli import record_main


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Record a LeRobot Dataset v3 from Go2 using the keyboard."
    )
    parser.add_argument("--interface", default="eth0", help="DDS network interface")
    parser.add_argument("--serial", required=True, help="RealSense camera serial number")
    parser.add_argument("--repo-id", required=True, help="Dataset id, for example name/go2_walk")
    parser.add_argument("--task", required=True, help="Task stored with every frame")
    parser.add_argument("--episodes", type=int, default=10)
    parser.add_argument("--episode-seconds", type=int, default=30)
    parser.add_argument("--reset-seconds", type=int, default=10)
    parser.add_argument("--fps", type=int, default=20)
    parser.add_argument("--vx", type=float, default=0.3, help="Forward/back speed in m/s")
    parser.add_argument("--vy", type=float, default=0.2, help="Lateral speed in m/s")
    parser.add_argument("--wz", type=float, default=0.5, help="Yaw speed in rad/s")
    parser.add_argument("--push-to-hub", action="store_true")
    return parser


def build_record_args(args: argparse.Namespace) -> list[str]:
    if args.episodes <= 0 or args.episode_seconds <= 0 or args.reset_seconds < 0 or args.fps <= 0:
        raise ValueError("episodes, episode-seconds, and fps must be positive")
    if args.vx <= 0 or args.vy <= 0 or args.wz <= 0:
        raise ValueError("vx, vy, and wz must be positive")

    camera = (
        "{front: {type: intelrealsense, "
        f"serial_number_or_name: {json.dumps(args.serial)}, width: 640, height: 480, fps: 30}}"
    )
    return [
        "--robot.type=unitree_go2",
        f"--robot.network_interface={args.interface}",
        f"--robot.control_frequency={args.fps}",
        f"--robot.cameras={camera}",
        "--teleop.type=unitree_go2_keyboard",
        f"--teleop.vx={args.vx}",
        f"--teleop.vy={args.vy}",
        f"--teleop.wz={args.wz}",
        # LeRobot reserves Q and R for recording controls.
        "--teleop.yaw_left_key=j",
        "--teleop.yaw_right_key=l",
        "--teleop.reset_key=u",
        f"--dataset.repo_id={args.repo_id}",
        f"--dataset.single_task={args.task}",
        f"--dataset.fps={args.fps}",
        f"--dataset.num_episodes={args.episodes}",
        f"--dataset.episode_time_s={args.episode_seconds}",
        f"--dataset.reset_time_s={args.reset_seconds}",
        "--dataset.no_stamp=true",
        f"--dataset.push_to_hub={'true' if args.push_to_hub else 'false'}",
    ]


def main(argv: Sequence[str] | None = None) -> None:
    args = build_parser().parse_args(argv)
    record_args = build_record_args(args)
    print("Keyboard: W/S forward, A/D lateral, J/L yaw, Space stop, X emergency stop, U reset")
    print(f"Recording {args.episodes} episode(s) to {args.repo_id}")

    original_argv = sys.argv
    try:
        sys.argv = ["lerobot-record", *record_args]
        record_main()
    finally:
        sys.argv = original_argv


if __name__ == "__main__":
    main()
