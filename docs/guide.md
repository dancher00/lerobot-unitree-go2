# LeRobot Unitree Go2

An open-source, ROS-free adapter that makes Unitree Go2 and Go2 EDU usable by Hugging Face
LeRobot's standard `lerobot-record` workflow. It records synchronized D435i RGB, compact body
state, base velocity action, task, and standard timestamps directly as a LeRobot Dataset v3.

> **Safety status:** mock mode and Dataset v3 round trips are tested in CI. Real SDK calls follow
> Unitree SDK2, but hardware behavior must be validated in a clear, controlled area before use.

## 1. What this project is

The package is a native LeRobot third-party robot and teleoperator plugin. Installation registers:

- `--robot.type=unitree_go2`
- `--teleop.type=unitree_go2_gamepad`
- `--teleop.type=unitree_go2_keyboard`

It uses `SportClient.Move(vx, vy, vyaw)` and `SportClient.StopMove()` directly. ROS is not in the
runtime or data path.

## 2. Supported hardware

- Unitree Go2 EDU (primary target; Go2 with SDK2 high-level sport service should also work)
- Intel RealSense D435i RGB; depth is supported through LeRobot's native RealSense camera
- Linux host with Ethernet recommended
- SDL-compatible gamepad or keyboard

The current LeRobot source (commit `89236ea`, reporting version 0.6.2) requires Python 3.12+ and is
pinned in `pyproject.toml` until that release reaches PyPI. The adapter code intentionally uses no
syntax newer than Python 3.10, but the effective runtime requirement follows LeRobot.

## 3. Architecture

```text
LeRobot recorder -> UnitreeGo2 -> Go2Backend -> SDK2 SportClient + DDS state
                         |             |
                         |             +-> independent command watchdog
                         +-> LeRobot RealSenseCamera -> D435i

Teleoperator -> {base.vx, base.vy, base.wz} -> clipped command -> Dataset v3 action
```

The hardware boundary is only five operations (`connect`, `read_state`, `send_velocity`, `stop`,
`disconnect`). `MockGo2Backend` implements the same boundary. Scalar names are namespaced with
`base.` so a future composite adapter can append arm and gripper fields without renaming today's
base data.

## 4. Installation

```bash
git clone https://github.com/dancher00/lerobot-unitree-go2.git
cd lerobot-unitree-go2
python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
pip install -e '.[test,gamepad,keyboard]'
```

Install RealSense support on the recording host:

```bash
pip install -e '.[realsense]'
lerobot-find-cameras realsense
```

SDK2 requires CycloneDDS. Follow the
[official SDK2 Python installation](https://github.com/unitreerobotics/unitree_sdk2_python), then:

```bash
pip install -e '.[unitree]'
```

The SDK extra is pinned to the SDK revision reviewed for this adapter. On platforms where its
`cyclonedds==0.10.2` wheel is unavailable, build CycloneDDS 0.10.x and set `CYCLONEDDS_HOME` as
described upstream before installing.

## 5. Unitree networking setup

Use a dedicated wired interface when possible. Configure it on the Go2 development subnet (the
usual robot-side address is `192.168.123.161`):

```bash
ip link show
sudo ip addr add 192.168.123.99/24 dev eth0
ping -c 3 192.168.123.161
```

Do not run another motion client simultaneously. Ensure the high-level sport service is enabled.
DDS discovery uses multicast; disable VPN/firewall rules that block multicast on the selected NIC.
The SDK2 channel factory is process-global, so one process should use one DDS interface/domain.

## 6. RealSense setup

Connect the D435i to USB 3 and find its stable serial number:

```bash
lerobot-find-cameras realsense
python examples/test_realsense.py --serial 123456789 --width 640 --height 480 --fps 30
python examples/test_realsense.py --serial 123456789 --depth
```

The adapter reuses LeRobot's `RealSenseCamera`, including its warm-up, buffered capture, age check,
and connect recovery. RGB is returned as `uint8` HWC in RGB order. Depth, when enabled, is `uint16`
millimeters and becomes `observation.images.front_depth`. Camera IMU streams are not yet exposed by
LeRobot's native RealSense abstraction; Go2 body IMU is available with `extended_state=true` and in
optional raw logs.

## 7. Test connection to Go2

This read-only example subscribes to state and never sends a movement command:

```bash
python examples/read_go2_state.py --interface eth0
```

If state times out, confirm the interface name, sport service, DDS multicast, and topic. Firmware
variants can override `--robot.state_topic` (default `rt/sportmodestate`).

## 8. Test camera

```bash
python examples/test_realsense.py --serial 123456789
```

A frame shape of `(480, 640, 3)` confirms the requested default profile.

## 9. Teleoperate Go2

Clear the area, keep the Unitree remote/app stop control available, and begin with the robot lifted
or speed limits reduced.

```bash
python examples/teleop_go2.py --interface eth0 --serial 123456789
python examples/teleop_go2.py --interface eth0 --serial 123456789 --keyboard
```

Keyboard controls follow ROS `teleop_twist_keyboard`: `I/,` forward/back, `J/L` yaw,
`U/O/M/.` arcs, Shift for holonomic strafing, `K` stop, `X` latched emergency stop, and `V` reset.
Gamepad: left stick forward/lateral, right-stick horizontal yaw, button 1
(usually B/Circle) latches emergency stop, button 7 (usually Start) resets it. Axes and buttons are
configurable.

## 10. Record the first LeRobot dataset

For keyboard control, the short recording command is:

```bash
lerobot-go2-keyboard-record \
  --interface eth0 \
  --serial 123456789 \
  --repo-id username/go2_dataset \
  --task 'Approach the red cylinder' \
  --episodes 5
```

Controls follow ROS `teleop_twist_keyboard`: `I/,` forward/back, `J/L` yaw, `U/O/M/.` arcs,
Shift for strafing, `K` stop, `X` emergency stop, and `V` reset. Recording keeps `Q` and `R`.
The same entry point is available as `python examples/record_go2_keyboard.py`.

Current LeRobot represents cameras as a native `cameras` mapping. This is the exact standard CLI:

```bash
lerobot-record \
  --robot.type=unitree_go2 \
  --robot.network_interface=eth0 \
  --robot.control_frequency=20 \
  --robot.cameras='{front: {type: intelrealsense, serial_number_or_name: 123456789, width: 640, height: 480, fps: 30, use_depth: false}}' \
  --teleop.type=unitree_go2_gamepad \
  --teleop.max_vx=0.5 \
  --teleop.max_vy=0.3 \
  --teleop.max_wz=0.8 \
  --dataset.repo_id=username/go2_dataset \
  --dataset.single_task='Approach the red cylinder' \
  --dataset.fps=20 \
  --dataset.num_episodes=5 \
  --dataset.episode_time_s=30 \
  --dataset.reset_time_s=15 \
  --dataset.no_stamp=true \
  --dataset.push_to_hub=false
```

`lerobot-go2-record` is only a convenience alias for that same upstream recorder. Keep teleoperator
gains at or below robot limits. The adapter clips again at the hardware boundary and updates the
action dictionary in place so current `lerobot-record` stores the exact command actually sent.

For hardware-free CLI validation, omit the camera map:

```bash
python examples/mock_robot.py
lerobot-record \
  --robot.type=unitree_go2 --robot.mock=true \
  --teleop.type=unitree_go2_keyboard \
  --dataset.repo_id=local/go2_mock --dataset.single_task='Mock walk' \
  --dataset.fps=20 --dataset.num_episodes=1 --dataset.episode_time_s=5 \
  --dataset.push_to_hub=false --dataset.video=false
```

## 11. Dataset schema

| Dataset field | Shape | Units / meaning |
|---|---:|---|
| `observation.images.front` | `(480, 640, 3)` | RGB `uint8` |
| `observation.state` | `(6,)` | `vx, vy` m/s; `wz, roll, pitch, yaw` rad/s or rad |
| `action` | `(3,)` | commanded `vx, vy` m/s and `wz` rad/s |
| `task` | scalar | LeRobot task index/text metadata |
| `timestamp` | scalar | standard LeRobot episode timestamp |

Feature names stored in metadata are `base.vx`, `base.vy`, `base.wz`, `base.roll`, `base.pitch`,
`base.yaw`. With `--robot.extended_state=true`, acceleration, angular velocity, height, four foot
forces, battery voltage, and 12 joint positions are appended to `observation.state`.

## 12. Visualize a dataset

```bash
lerobot-dataset-viz --repo-id username/go2_dataset --episode-index 0
```

For a non-default local root, add `--root /path/to/root --mode local`.

## 13. Push to Hugging Face Hub

Authenticate with `hf auth login` and record with `--dataset.push_to_hub=true`, or load and push an
existing dataset:

```python
from lerobot.datasets import LeRobotDataset

dataset = LeRobotDataset("username/go2_dataset")
dataset.push_to_hub()
```

## 14. Safety notes

- Never test around people, stairs, traffic, glass, or loose cables.
- Start with low limits and keep a physical/Unitree remote stop available.
- Commands are clipped to `max_vx=0.5`, `max_vy=0.3`, `max_wz=0.8` by default.
- Releasing controls produces zero. Exceptions, stale state, disconnect, Ctrl+C cleanup, and the
  independent 0.5 s command watchdog issue stop commands.
- SDK stop sends both `Move(0, 0, 0)` and `StopMove()` as independent attempts.
- This software is not a certified safety system.

## 15. Troubleshooting

- **No state:** verify `ping`, interface name, multicast, sport mode, and `state_topic`.
- **Camera timeout:** use USB 3, reduce resolution/FPS, close other camera users, inspect dmesg.
- **Plugin type unknown:** confirm `pip show lerobot_robot_unitree_go2`; editable installs are supported.
- **Loop warnings:** set camera and dataset FPS to compatible values; reduce visualization/video load.
- **Gamepad missing:** check `python -m pygame.examples.joystick`; configure axis/button indices.
- **Robot stops after 0.5 s:** the control loop is too slow or stopped feeding the watchdog; fix the
  stall instead of increasing the timeout first.

## 16. Add more sensors

Add scalar values to `Go2State` and the matching ordered feature map, or add a LeRobot `Camera`
implementation to `config.cameras`. Keep acquisition out of dataset code: `UnitreeGo2` should expose
flat hardware values and let LeRobot build Dataset v3 features and timestamps.

Optional full diagnostics can be written without changing the learned schema:

```bash
--robot.raw_log_path=raw_logs/session.jsonl --robot.extended_state=true
```

## 17. Roadmap

- Hardware validation matrix across Go2 EDU firmware versions
- Expose D435i motion streams when upstream LeRobot camera features support them
- Add gamepad mapping presets and rumble feedback
- Contribute a recorder fix upstream so all robots persist `send_action()`'s return value directly
- Future composite Go2 + SO-101 adapter with separate base/arm namespaces and wrist camera

## Development

```bash
pip install -e '.[test]'
ruff check .
pytest
```

See [CONTRIBUTING.md](../CONTRIBUTING.md). Licensed under Apache-2.0.
