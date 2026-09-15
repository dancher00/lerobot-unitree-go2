"""Gamepad teleoperator configuration."""

from dataclasses import dataclass

from lerobot.teleoperators import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("unitree_go2_gamepad")
@dataclass
class UnitreeGo2GamepadTeleopConfig(TeleoperatorConfig):
    max_vx: float = 0.5
    max_vy: float = 0.3
    max_wz: float = 0.8
    deadzone: float = 0.12
    gamepad_index: int = 0
    estop_button: int = 1
    reset_estop_button: int = 7
    forward_axis: int = 1
    lateral_axis: int = 0
    yaw_axis: int = 2

    def __post_init__(self) -> None:
        if self.max_vx <= 0 or self.max_vy <= 0 or self.max_wz <= 0:
            raise ValueError("Gamepad velocity gains must be positive")
        if not 0 <= self.deadzone < 1:
            raise ValueError("deadzone must be in [0, 1)")
        if (
            min(
                self.gamepad_index,
                self.estop_button,
                self.reset_estop_button,
                self.forward_axis,
                self.lateral_axis,
                self.yaw_axis,
            )
            < 0
        ):
            raise ValueError("Gamepad indices must be non-negative")
