"""Validate local LeRobot v3 tables, episode ranges and every video frame."""

import argparse
import json
from pathlib import Path

import av
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq


def validate(root):
    info = json.loads((root / "meta/info.json").read_text())
    data = pa.concat_tables([pq.read_table(p) for p in sorted((root / "data").rglob("*.parquet"))]).to_pandas()
    episodes = pa.concat_tables([pq.read_table(p) for p in sorted((root / "meta/episodes").rglob("*.parquet"))]).to_pandas()
    data = data.sort_values("index")
    episodes = episodes.sort_values("episode_index")
    assert len(data) == info["total_frames"]
    assert len(episodes) == info["total_episodes"]
    np.testing.assert_array_equal(data["index"], np.arange(len(data)))
    np.testing.assert_array_equal(episodes["episode_index"], np.arange(len(episodes)))
    for key in ("action", "observation.state"):
        values = np.stack(data[key])
        assert values.shape == (len(data), *info["features"][key]["shape"])
        assert np.isfinite(values).all()
    assert set(data["task_index"]) <= set(pq.read_table(root / "meta/tasks.parquet")["task_index"].to_pylist())
    video_times = {}
    for path in sorted((root / "videos").rglob("*.mp4")):
        with av.open(str(path)) as container:
            times = []
            for frame in container.decode(video=0):
                assert (frame.height, frame.width) == (480, 640)
                times.append(float(frame.time))
        assert times and np.all(np.diff(times) > 0)
        video_times[path.relative_to(root).as_posix()] = np.array(times)
    cursor = 0
    for _, ep in episodes.iterrows():
        start, end = int(ep["dataset_from_index"]), int(ep["dataset_to_index"])
        assert start == cursor and end - start == ep["length"]
        rows = data.iloc[start:end]
        assert (rows["episode_index"] == ep["episode_index"]).all()
        np.testing.assert_array_equal(rows["frame_index"], np.arange(len(rows)))
        np.testing.assert_allclose(rows["timestamp"], np.arange(len(rows)) / info["fps"], atol=1e-5)
        table_path = info["data_path"].format(chunk_index=int(ep["data/chunk_index"]), file_index=int(ep["data/file_index"]))
        table_indices = pq.read_table(root / table_path, columns=["index"])["index"].to_numpy()
        assert np.isin(rows["index"], table_indices).all()
        for key, feature in info["features"].items():
            if feature["dtype"] != "video":
                continue
            prefix = "videos/" + key
            path = info["video_path"].format(video_key=key, chunk_index=int(ep[prefix + "/chunk_index"]), file_index=int(ep[prefix + "/file_index"]))
            times = video_times[path]
            targets = rows["timestamp"].to_numpy() + ep[prefix + "/from_timestamp"]
            nearest = np.searchsorted(times, targets).clip(0, len(times) - 1)
            error = np.minimum(np.abs(times[nearest] - targets), np.abs(times[np.maximum(nearest - 1, 0)] - targets))
            assert np.max(error) < 1 / info["fps"] / 2
        cursor = end
    assert cursor == len(data)
    actions = np.stack(data["action"])
    return dict(episodes=len(episodes), frames=len(data), fps=info["fps"],
                duration_seconds=len(data) / info["fps"], video_files=len(video_times),
                decoded_video_frames=sum(map(len, video_times.values())),
                episode_length_min=int(episodes["length"].min()),
                episode_length_max=int(episodes["length"].max()),
                action_min=actions.min(axis=0).tolist(), action_max=actions.max(axis=0).tolist(),
                zero_action_fraction=float(np.mean(np.all(actions == 0, axis=1))),
                status="passed")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    args = parser.parse_args()
    print(json.dumps(validate(args.root), indent=2))
