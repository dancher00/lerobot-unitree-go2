"""LeRobot configuration for Unitree Go2 and Go2 EDU."""

from dataclasses import dataclass, field
from pathlib import Path

from lerobot.cameras import CameraConfig
from lerobot.robots import RobotConfig


@RobotConfig.register_subclass("unitree_go2")
@dataclass
class UnitreeGo2Config(RobotConfig):
    """Configuration used by :class:`UnitreeGo2`.

    Camera configuration follows LeRobot's native convention. A typical real
    setup has one ``intelrealsense`` camera named ``front``.
    """

    network_interface: str = "eth0"
    domain_id: int = 0
    state_topic: str = "rt/sportmodestate"
    low_state_topic: str = "rt/lowstate"
    extended_state: bool = False
    control_frequency: int = 20

    max_vx: float = 0.5
    max_vy: float = 0.3
    max_wz: float = 0.8
    watchdog_timeout_s: float = 0.5
    state_timeout_s: float = 1.0
    connect_timeout_s: float = 5.0

    cameras: dict[str, CameraConfig] = field(default_factory=dict)
    camera_max_age_ms: int = 500
    camera_reconnect: bool = True
    camera_reconnect_attempts: int = 1

    mock: bool = False
    mock_camera: bool = True
    mock_camera_width: int = 640
    mock_camera_height: int = 480
    mock_response_rate: float = 8.0

    raw_log_path: Path | None = None

    def __post_init__(self) -> None:
        super().__post_init__()
        if not self.network_interface.strip():
            raise ValueError("network_interface must not be empty")
        if not self.state_topic.strip() or not self.low_state_topic.strip():
            raise ValueError("DDS topic names must not be empty")
        if not 1 <= self.control_frequency <= 200:
            raise ValueError("control_frequency must be in [1, 200] Hz")
        for name in ("max_vx", "max_vy", "max_wz", "watchdog_timeout_s", "state_timeout_s"):
            if getattr(self, name) <= 0:
                raise ValueError(f"{name} must be positive")
        if self.connect_timeout_s <= 0:
            raise ValueError("connect_timeout_s must be positive")
        if self.camera_max_age_ms <= 0:
            raise ValueError("camera_max_age_ms must be positive")
        if self.camera_reconnect_attempts < 0:
            raise ValueError("camera_reconnect_attempts must be non-negative")
        if self.mock_camera_width <= 0 or self.mock_camera_height <= 0:
            raise ValueError("mock camera dimensions must be positive")
        if self.mock_response_rate <= 0:
            raise ValueError("mock_response_rate must be positive")
        if not self.mock and "front" not in self.cameras:
            raise ValueError(
                "A real Go2 configuration requires a camera named 'front'. "
                "Use --robot.cameras='{front: {...}}'."
            )
