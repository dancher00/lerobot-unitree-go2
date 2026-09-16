---
license: cc-by-4.0
language:
- en
tags:
- lerobot
- robotics
- unitree
- go2
- imitation-learning
task_categories:
- robotics
size_categories:
- 10K<n<100K
---

# Go2 Object Approach v1

Keyboard-teleoperated Unitree Go2 EDU trajectories for the task:
"Approach the target object and stop in a manipulation-ready pose."

The collection contains 202 episodes and 48,889 frames at a nominal 20 Hz
(2,444.45 seconds, approximately 40.74 minutes). Episode lengths range from
109 to 502 frames (5.45–25.10 seconds). Data is stored in LeRobot Dataset v3
with Parquet telemetry and H.264 front-camera video. No audio or depth is included.

This is a Go2-only approach dataset. It does not contain arm actions, wrist
images, or pick-and-place demonstrations. The associated code repository also
provides an experimental Go2 + SO-101 bridge:
https://github.com/dancher00/lerobot-unitree-go2

## Hardware and collection

Go2 EDU was controlled using keyboard velocity commands. An onboard RealSense
D435i provided RGB images; base state came from Unitree SDK2 SportModeState.
The collection uses a front camera facing a tabletop target in an indoor
workspace. Sampled images show a cylinder and a cup on the table.

Images and telemetry were transported to a laptop over SSH. Camera acquisition
was requested at 30 FPS, while the dataset uses a 20 Hz grid and the latest
available image. JPEG was used for network transport and H.264 for stored video.
Dataset timestamps are nominal frame indices divided by 20, not camera exposure
timestamps. Raw session logs are not distributed in this release.

## Features

| Feature | Shape | Meaning and units |
| --- | --- | --- |
| `observation.images.front` | 480 × 640 × 3 | RGB video; LeRobot loads CHW float tensors |
| `observation.state` | 6 | vx, vy (m/s), yaw speed (rad/s), roll, pitch, yaw (rad), as reported by SDK2 |
| `action` | 3 | base.vx, base.vy (m/s), base.wz (rad/s): base velocity commands |
| `timestamp` | 1 | Episode-relative nominal time in seconds |
| `episode_index`, `frame_index`, `index`, `task_index` | 1 each | LeRobot indices |

Recorded action ranges are ±0.30 m/s for vx/vy and ±0.60 rad/s for yaw speed.
Approximately 52.55% of frames have all three action components equal to zero.
These are recorded zero commands; they do not establish that measured robot
velocity is zero.

## Loading

Load the published dataset from [Hugging Face](https://huggingface.co/datasets/dancher00/go2_object_approach_v1):

```python
from lerobot.datasets.lerobot_dataset import LeRobotDataset

dataset = LeRobotDataset("dancher00/go2_object_approach_v1", video_backend="pyav")
sample = dataset[0]
print(sample["observation.state"], sample["action"])
```

For a local copy, pass `root="/path/to/go2_object_approach_v1"`. Validation used
LeRobot 0.6.2 at upstream commit `b6ec0060779550c0a157ae34feb89e0cf86012a8`.

## Validation and limitations

Release preparation on 2026-09-16 checked every table row for valid indices,
finite state/actions and feature dimensions; contiguous episode ranges and
nominal timestamps; every video's full decode; and video timestamp coverage
for all recorded observations. All 28 videos decoded, totaling 48,889 frames.
The LeRobot loader also decoded the first, middle and last frame of every
episode (606 samples). This verifies structural readability, not task success.

There are no per-episode success labels, rewards, or independent evaluations of
policy performance. The supplied split is `train: 0:202`; no held-out test set
is provided. The collection represents one recording setup and should not be
treated as evidence of generalization. Repeated pauses/zero commands and camera
motion may affect training. Inspect trajectories and define an appropriate
evaluation split before reporting results.

The validation can be repeated from the code checkout:

```bash
python scripts/validate_dataset.py /path/to/go2_object_approach_v1
```

## License and attribution

Dataset: Creative Commons Attribution 4.0 International (CC BY 4.0).
Credit `dancher00, Go2 Object Approach v1 (2026)` and link to the dataset when
redistributing or using it in published work. Code in the associated repository
is separately licensed under Apache-2.0.
