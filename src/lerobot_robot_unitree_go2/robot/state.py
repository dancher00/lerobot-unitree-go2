"""Typed, SDK-independent Go2 state."""

from dataclasses import asdict, dataclass, field


@dataclass(frozen=True, slots=True)
class Go2State:
    """A snapshot of Go2 state in SI units."""

    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    linear_acceleration: tuple[float, float, float] = (0.0, 0.0, 0.0)
    angular_velocity: tuple[float, float, float] = (0.0, 0.0, 0.0)
    body_height: float = 0.0
    foot_contacts: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    battery_voltage: float = 0.0
    joint_positions: tuple[float, ...] = (0.0,) * 12
    source_timestamp_s: float | None = None
    received_monotonic_s: float = 0.0
    extra: dict[str, object] = field(default_factory=dict)

    def minimal_observation(self) -> dict[str, float]:
        """Return the stable six-element LeRobot state schema."""
        return {
            "base.vx": float(self.vx),
            "base.vy": float(self.vy),
            "base.wz": float(self.wz),
            "base.roll": float(self.roll),
            "base.pitch": float(self.pitch),
            "base.yaw": float(self.yaw),
        }

    def extended_observation(self) -> dict[str, float]:
        """Return optional diagnostics as flat state features."""
        values = self.minimal_observation()
        values.update(
            {
                **{
                    f"base.linear_acceleration.{axis}": float(value)
                    for axis, value in zip("xyz", self.linear_acceleration, strict=True)
                },
                **{
                    f"base.angular_velocity.{axis}": float(value)
                    for axis, value in zip("xyz", self.angular_velocity, strict=True)
                },
                "base.height": float(self.body_height),
                **{
                    f"base.foot_contact.{index}": float(value)
                    for index, value in enumerate(self.foot_contacts)
                },
                "base.battery_voltage": float(self.battery_voltage),
                **{
                    f"base.joint.{index}.position": float(value)
                    for index, value in enumerate(self.joint_positions)
                },
            }
        )
        return values

    def to_dict(self) -> dict[str, object]:
        return asdict(self)
