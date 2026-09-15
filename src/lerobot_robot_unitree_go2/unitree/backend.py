"""SDK-neutral backends for the Go2 adapter."""

from __future__ import annotations

import math
import threading
import time
from abc import ABC, abstractmethod

from ..robot.state import Go2State


class Go2Backend(ABC):
    """The narrow hardware boundary used by the LeRobot adapter."""

    @property
    @abstractmethod
    def is_connected(self) -> bool: ...

    @abstractmethod
    def connect(self) -> None: ...

    @abstractmethod
    def read_state(self) -> Go2State: ...

    @abstractmethod
    def send_velocity(self, vx: float, vy: float, wz: float) -> None: ...

    @abstractmethod
    def stop(self) -> None: ...

    @abstractmethod
    def disconnect(self) -> None: ...


class MockGo2Backend(Go2Backend):
    """Deterministic first-order velocity simulation for tests and CI."""

    def __init__(self, response_rate: float = 8.0) -> None:
        self.response_rate = response_rate
        self._connected = False
        self._lock = threading.Lock()
        self._target = [0.0, 0.0, 0.0]
        self._velocity = [0.0, 0.0, 0.0]
        self._rpy = [0.0, 0.0, 0.0]
        self._last_update = time.monotonic()
        self.command_history: list[tuple[float, float, float]] = []

    @property
    def is_connected(self) -> bool:
        return self._connected

    def connect(self) -> None:
        if self._connected:
            raise RuntimeError("Mock Go2 backend is already connected")
        self._last_update = time.monotonic()
        self._connected = True

    def _update(self, now: float) -> None:
        dt = max(0.0, now - self._last_update)
        alpha = 1.0 - math.exp(-self.response_rate * dt)
        for i in range(3):
            self._velocity[i] += alpha * (self._target[i] - self._velocity[i])
        self._rpy[2] += self._velocity[2] * dt
        self._last_update = now

    def read_state(self) -> Go2State:
        if not self._connected:
            raise RuntimeError("Mock Go2 backend is not connected")
        with self._lock:
            now = time.monotonic()
            self._update(now)
            return Go2State(
                vx=self._velocity[0],
                vy=self._velocity[1],
                wz=self._velocity[2],
                roll=self._rpy[0],
                pitch=self._rpy[1],
                yaw=self._rpy[2],
                received_monotonic_s=now,
            )

    def send_velocity(self, vx: float, vy: float, wz: float) -> None:
        if not self._connected:
            raise RuntimeError("Mock Go2 backend is not connected")
        with self._lock:
            self._update(time.monotonic())
            command = (float(vx), float(vy), float(wz))
            self._target[:] = command
            self.command_history.append(command)

    def stop(self) -> None:
        if not self._connected:
            return
        self.send_velocity(0.0, 0.0, 0.0)

    def disconnect(self) -> None:
        if not self._connected:
            return
        self.stop()
        self._connected = False
