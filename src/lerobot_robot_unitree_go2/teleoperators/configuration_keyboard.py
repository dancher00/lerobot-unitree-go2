"""Keyboard teleoperator configuration."""

from dataclasses import dataclass

from lerobot.teleoperators import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("unitree_go2_keyboard")
@dataclass
class UnitreeGo2KeyboardTeleopConfig(TeleoperatorConfig):
    vx: float = 0.5
    vy: float = 0.3
    wz: float = 0.8
    forward_key: str = "i"
    backward_key: str = ","
    left_key: str = "J"
    right_key: str = "L"
    yaw_left_key: str = "j"
    yaw_right_key: str = "l"
    stop_key: str = "k"
    estop_key: str = "x"
    reset_key: str = "v"

    def __post_init__(self) -> None:
        if self.vx <= 0 or self.vy <= 0 or self.wz <= 0:
            raise ValueError("Keyboard velocity gains must be positive")
        keys = [
            self.forward_key,
            self.backward_key,
            self.left_key,
            self.right_key,
            self.yaw_left_key,
            self.yaw_right_key,
            self.stop_key,
            self.estop_key,
            self.reset_key,
        ]
        if any(len(key) != 1 for key in keys) or len(set(keys)) != len(keys):
            raise ValueError("Keyboard control keys must be unique single characters")
