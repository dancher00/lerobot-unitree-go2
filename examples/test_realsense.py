#!/usr/bin/env python3
"""Open a D435i through LeRobot's native RealSense camera."""

import argparse

from lerobot.cameras.realsense import RealSenseCamera, RealSenseCameraConfig


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--serial", required=True)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--depth", action="store_true")
    args = parser.parse_args()
    camera = RealSenseCamera(
        RealSenseCameraConfig(
            serial_number_or_name=args.serial,
            width=args.width,
            height=args.height,
            fps=args.fps,
            use_depth=args.depth,
        )
    )
    camera.connect()
    try:
        color = camera.async_read()
        print(f"RGB: shape={color.shape}, dtype={color.dtype}")
        if args.depth:
            depth = camera.async_read_depth()
            print(f"Depth: shape={depth.shape}, dtype={depth.dtype}")
    finally:
        camera.disconnect()


if __name__ == "__main__":
    main()
