"""Analog gamepad control for Go2."""

from __future__ import annotations

from typing import Any

from lerobot.lerobot_types import RobotAction
from lerobot.teleoperators import Teleoperator
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from .configuration_gamepad import UnitreeGo2GamepadTeleopConfig


def apply_deadzone(value: float, deadzone: float) -> float:
    """Continuous rescaling outside a symmetric stick deadzone."""
    magnitude = abs(value)
    if magnitude <= deadzone:
        return 0.0
    scaled = (magnitude - deadzone) / (1.0 - deadzone)
    return max(-1.0, min(1.0, scaled if value >= 0 else -scaled))


class UnitreeGo2GamepadTeleop(Teleoperator):
    config_class = UnitreeGo2GamepadTeleopConfig
    name = "unitree_go2_gamepad"

    def __init__(self, config: UnitreeGo2GamepadTeleopConfig) -> None:
        super().__init__(config)
        self.config = config
        self._pygame: Any | None = None
        self._joystick: Any | None = None
        self._estop_latched = False

    @property
    def action_features(self) -> dict[str, type]:
        return {"base.vx": float, "base.vy": float, "base.wz": float}

    @property
    def feedback_features(self) -> dict:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._joystick is not None

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None: ...

    def configure(self) -> None: ...

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        del calibrate
        try:
            import pygame
        except ImportError as exc:
            raise ImportError("Gamepad teleoperation requires pygame") from exc
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() <= self.config.gamepad_index:
            pygame.quit()
            raise ConnectionError(f"No gamepad found at index {self.config.gamepad_index}")
        joystick = pygame.joystick.Joystick(self.config.gamepad_index)
        joystick.init()
        required_axis = max(
            self.config.forward_axis, self.config.lateral_axis, self.config.yaw_axis
        )
        required_button = max(self.config.estop_button, self.config.reset_estop_button)
        if joystick.get_numaxes() <= required_axis or joystick.get_numbuttons() <= required_button:
            joystick.quit()
            pygame.quit()
            raise ValueError("Gamepad does not expose the configured axes/buttons")
        self._pygame = pygame
        self._joystick = joystick

    @check_if_not_connected
    def get_action(self) -> RobotAction:
        self._pygame.event.pump()
        if self._joystick.get_button(self.config.estop_button):
            self._estop_latched = True
        if self._joystick.get_button(self.config.reset_estop_button):
            self._estop_latched = False
        if self._estop_latched:
            return {"base.vx": 0.0, "base.vy": 0.0, "base.wz": 0.0}
        forward = -apply_deadzone(
            float(self._joystick.get_axis(self.config.forward_axis)), self.config.deadzone
        )
        lateral = -apply_deadzone(
            float(self._joystick.get_axis(self.config.lateral_axis)), self.config.deadzone
        )
        yaw = -apply_deadzone(
            float(self._joystick.get_axis(self.config.yaw_axis)), self.config.deadzone
        )
        return {
            "base.vx": forward * self.config.max_vx,
            "base.vy": lateral * self.config.max_vy,
            "base.wz": yaw * self.config.max_wz,
        }

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        del feedback

    def disconnect(self) -> None:
        self._estop_latched = True
        if self._joystick is not None:
            self._joystick.quit()
            self._joystick = None
        if self._pygame is not None:
            self._pygame.quit()
            self._pygame = None
