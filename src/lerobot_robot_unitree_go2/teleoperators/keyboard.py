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
        return char.lower() if isinstance(char, str) else key

    def _on_press(self, key: object) -> None:
        normalised = self._normalise_key(key)
        self._pressed.add(normalised)
        if normalised == self.config.estop_key:
            self._estop_latched = True
        elif normalised == self.config.reset_key:
            self._estop_latched = False

    def _on_release(self, key: object) -> None:
        self._pressed.discard(self._normalise_key(key))

    @check_if_not_connected
    def get_action(self) -> RobotAction:
        keys = self._pressed.copy()
        space = self._keyboard.Key.space if self._keyboard is not None else object()
        if self._estop_latched or space in keys:
            return {"base.vx": 0.0, "base.vy": 0.0, "base.wz": 0.0}
        vx = self.config.vx * (
            float(self.config.forward_key in keys) - float(self.config.backward_key in keys)
        )
        vy = self.config.vy * (
            float(self.config.left_key in keys) - float(self.config.right_key in keys)
        )
        wz = self.config.wz * (
            float(self.config.yaw_left_key in keys) - float(self.config.yaw_right_key in keys)
        )
        return {"base.vx": vx, "base.vy": vy, "base.wz": wz}

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        del feedback

    def disconnect(self) -> None:
        self._pressed.clear()
        self._estop_latched = True
        if self._listener is not None:
            self._listener.stop()
            self._listener = None
