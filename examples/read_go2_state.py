#!/usr/bin/env python3
"""Print compact state without opening a camera or commanding motion."""

import argparse
import time

from lerobot_robot_unitree_go2.unitree import UnitreeSdk2Backend


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface", default="eth0")
    parser.add_argument("--topic", default="rt/sportmodestate")
    args = parser.parse_args()
    backend = UnitreeSdk2Backend(args.interface, state_topic=args.topic)
    backend.connect()
    try:
        while True:
            print(backend.read_state().minimal_observation())
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        backend.disconnect()


if __name__ == "__main__":
    main()
