"""Third-party LeRobot plugin for Unitree Go2 / Go2 EDU."""

from .robot import Go2State, UnitreeGo2, UnitreeGo2Config
from .teleoperators import (
    UnitreeGo2GamepadTeleop,
    UnitreeGo2GamepadTeleopConfig,
    UnitreeGo2KeyboardTeleop,
    UnitreeGo2KeyboardTeleopConfig,
)

__version__ = "0.1.0"

__all__ = [
    "Go2State",
    "UnitreeGo2",
    "UnitreeGo2Config",
    "UnitreeGo2GamepadTeleop",
    "UnitreeGo2GamepadTeleopConfig",
    "UnitreeGo2KeyboardTeleop",
    "UnitreeGo2KeyboardTeleopConfig",
]
