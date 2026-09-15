"""Generated RGB camera used by mock mode."""

import time

import numpy as np


class GeneratedCamera:
    """Small subset of LeRobot's Camera protocol needed by Robot implementations."""

    use_rgb = True
    use_depth = False

    def __init__(self, width: int, height: int) -> None:
        self.width = width
        self.height = height
        self.fps = 30
        self.latest_timestamp: float | None = None
        self._connected = False
        x = np.linspace(0, 255, width, dtype=np.uint8)
        y = np.linspace(0, 255, height, dtype=np.uint8)[:, None]
        self._base = np.empty((height, width, 3), dtype=np.uint8)
        self._base[..., 0] = x
        self._base[..., 1] = y
        self._base[..., 2] = 96

    @property
    def is_connected(self) -> bool:
        return self._connected

    def connect(self) -> None:
        if self._connected:
            raise RuntimeError("Generated camera is already connected")
        self._connected = True

    def read_latest(self, max_age_ms: int = 500) -> np.ndarray:
        del max_age_ms
        if not self._connected:
            raise RuntimeError("Generated camera is not connected")
        self.latest_timestamp = time.monotonic()
        frame = self._base.copy()
        marker = int(self.latest_timestamp * 30) % self.width
        frame[:, marker : min(marker + 4, self.width)] = (255, 255, 255)
        return frame

    def disconnect(self) -> None:
        self._connected = False
