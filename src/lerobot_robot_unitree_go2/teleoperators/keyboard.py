"""WASD keyboard control for Go2."""

from __future__ import annotations

from typing import Any

from lerobot.lerobot_types import RobotAction
from lerobot.teleoperators import Teleoperator
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from .configuration_keyboard import UnitreeGo2KeyboardTeleopConfig


class UnitreeGo2KeyboardTeleop(Teleoperator):
    """Hold-to-move keyboard teleoperator; release always produces zero."""

    config_class = UnitreeGo2KeyboardTeleopConfig
    name = "unitree_go2_keyboard"

    def __init__(self, config: UnitreeGo2KeyboardTeleopConfig) -> None:
        super().__init__(config)
        self.config = config
        self._listener: Any | None = None
        self._keyboard: Any | None = None
        self._pressed: set[object] = set()
        self._estop_latched = False

    @property
    def action_features(self) -> dict[str, type]:
        return {"base.vx": float, "base.vy": float, "base.wz": float}

    @property
    def feedback_features(self) -> dict:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._listener is not None and self._listener.is_alive()

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None: ...

    def configure(self) -> None: ...

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        del calibrate
        try:
            from pynput import keyboard
        except ImportError as exc:
            raise ImportError("Keyboard teleoperation requires pynput") from exc
        self._keyboard = keyboard
        self._listener = keyboard.Listener(on_press=self._on_press, on_release=self._on_release)
        self._listener.start()

    @staticmethod
    def _normalise_key(key: object) -> object:
        char = getattr(key, "char", None)
        return char if isinstance(char, str) else key

    def _on_press(self, key: object) -> None:
        normalised = self._normalise_key(key)
        self._pressed.add(normalised)
        comparable = normalised.lower() if isinstance(normalised, str) else normalised
        if comparable == self.config.estop_key.lower():
            self._estop_latched = True
        elif comparable == self.config.reset_key.lower():
            self._estop_latched = False

    def _on_release(self, key: object) -> None:
        normalised = self._normalise_key(key)
        self._pressed.discard(normalised)
        if isinstance(normalised, str):
            self._pressed.discard(normalised.lower())
            self._pressed.discard(normalised.upper())

    @check_if_not_connected
    def get_action(self) -> RobotAction:
        keys = self._pressed.copy()
        space = self._keyboard.Key.space if self._keyboard is not None else object()
        stop_pressed = self.config.stop_key in keys or self.config.stop_key.upper() in keys
        if self._estop_latched or space in keys or stop_pressed:
            return {"base.vx": 0.0, "base.vy": 0.0, "base.wz": 0.0}
        direction = [
            float(self.config.forward_key in keys) - float(self.config.backward_key in keys),
            float(self.config.left_key in keys) - float(self.config.right_key in keys),
            float(self.config.yaw_left_key in keys) - float(self.config.yaw_right_key in keys),
        ]
        # Canonical teleop_twist_keyboard diagonals and shifted holonomic bindings.
        ros_bindings = {
            "u": (1.0, 0.0, 1.0),
            "o": (1.0, 0.0, -1.0),
            "m": (-1.0, 0.0, -1.0),
            ".": (-1.0, 0.0, 1.0),
            "I": (1.0, 0.0, 0.0),
            "<": (-1.0, 0.0, 0.0),
            "U": (1.0, 1.0, 0.0),
            "O": (1.0, -1.0, 0.0),
            "M": (-1.0, 1.0, 0.0),
            ">": (-1.0, -1.0, 0.0),
        }
        for key in keys:
            if key in ros_bindings:
                for index, value in enumerate(ros_bindings[key]):
                    direction[index] += value
        vx = self.config.vx * max(-1.0, min(1.0, direction[0]))
        vy = self.config.vy * max(-1.0, min(1.0, direction[1]))
        wz = self.config.wz * max(-1.0, min(1.0, direction[2]))
        return {"base.vx": vx, "base.vy": vy, "base.wz": wz}

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        del feedback

    def disconnect(self) -> None:
        self._pressed.clear()
        self._estop_latched = True
        if self._listener is not None:
            self._listener.stop()
            self._listener = None
