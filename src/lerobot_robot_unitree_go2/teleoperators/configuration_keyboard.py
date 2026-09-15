"""Keyboard teleoperator configuration."""

from dataclasses import dataclass

from lerobot.teleoperators import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("unitree_go2_keyboard")
@dataclass
class UnitreeGo2KeyboardTeleopConfig(TeleoperatorConfig):
    vx: float = 0.5
    vy: float = 0.3
    wz: float = 0.8

    def __post_init__(self) -> None:
        if self.vx <= 0 or self.vy <= 0 or self.wz <= 0:
            raise ValueError("Keyboard velocity gains must be positive")
