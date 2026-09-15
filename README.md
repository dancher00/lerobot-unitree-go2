<div align="center">

<img src="docs/assets/go2-lerobot-hero.png" alt="Unitree Go2 × LeRobot — real D435i capture" width="100%">

# LeRobot × Unitree Go2

**Turn a Go2 into a native Hugging Face LeRobot Dataset v3 recorder — without ROS.**

[![CI](https://github.com/dancher00/lerobot-unitree-go2/actions/workflows/ci.yml/badge.svg)](https://github.com/dancher00/lerobot-unitree-go2/actions/workflows/ci.yml)
[![Apache-2.0](https://img.shields.io/badge/license-Apache--2.0-35baf6.svg)](LICENSE)
[![Python 3.12](https://img.shields.io/badge/python-3.12-44ff9f.svg)](pyproject.toml)
[![LeRobot Dataset v3](https://img.shields.io/badge/LeRobot-Dataset_v3-ffcc4d.svg)](https://huggingface.co/docs/lerobot)
[![ROS-free](https://img.shields.io/badge/ROS-free-b084ff.svg)](#why)

[Quick start](#quick-start) · [Record](#record) · [Full guide](docs/guide.md) · [Examples](examples)

</div>

## Why

A small, production-minded bridge between **Unitree SDK2** and the standard
`lerobot-record` workflow:

- 🐕 Go2 / Go2 EDU base control: `[vx, vy, wz]`
- 📷 RealSense D435i RGB through LeRobot's native camera stack
- 🎮 Gamepad + keyboard teleoperation
- 🛑 clipping, watchdog, emergency stop, zero-on-exit
- 🧪 mock robot + Dataset v3 round-trip tests in CI
- 🚫 no ROS in the control or data path

### Real hardware proof

<div align="center">
<img src="docs/assets/go2-walk-real.gif" alt="Real Go2 EDU walking while recording D435i and telemetry" width="720">
<br>
<sub>Real Go2 EDU · D435i · SDK2 · 0.30 m/s command · synchronized at 20 Hz</sub>
</div>

> Validated live: stand-up, DDS state, velocity control, safe stop, 640×480 RGB,
> and a **190-frame LeRobot Dataset v3** that reopens with standard tooling.

## Quick start

```bash
git clone https://github.com/dancher00/lerobot-unitree-go2.git
cd lerobot-unitree-go2
python3.12 -m venv .venv && source .venv/bin/activate
pip install -e '.[test,unitree,realsense,gamepad,keyboard]'
pytest -q
```

Find the camera and verify read-only Go2 state:

```bash
lerobot-find-cameras realsense
python examples/read_go2_state.py --interface eth0
```

## Record

```bash
lerobot-record \
  --robot.type=unitree_go2 \
  --robot.network_interface=eth0 \
  --robot.cameras='{front: {type: intelrealsense, serial_number_or_name: 243722071463, width: 640, height: 480, fps: 30}}' \
  --teleop.type=unitree_go2_gamepad \
  --dataset.repo_id=YOUR_NAME/go2_walk \
  --dataset.single_task='Walk to the red object' \
  --dataset.fps=20 \
  --dataset.num_episodes=10
```

Keyboard fallback:

```bash
python examples/teleop_go2.py --interface eth0 --serial 243722071463 --keyboard
# W/S forward · A/D lateral · Q/E yaw · Space stop · X emergency stop
```

Every frame is standard LeRobot data:

```text
observation.images.front   RGB, 640×480
observation.state          [vx, vy, wz, roll, pitch, yaw]
action                     [vx, vy, wz]
task · timestamp · episode_index · frame_index
```

Visualize and share:

```bash
lerobot-dataset-viz --repo-id YOUR_NAME/go2_walk --episode-index 0
hf auth login  # then record with --dataset.push_to_hub=true
```

> [!CAUTION]
> Clear the area, keep the Unitree remote stop available, and start with low limits.
> This is research software, not a certified safety system.

<div align="center">

[Full setup & troubleshooting →](docs/guide.md)

If this saves you a weekend, **give it a ⭐** — it helps the project find the next robot builder.

Apache-2.0

</div>
