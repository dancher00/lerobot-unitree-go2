"""Real Go2 backend using Unitree SDK2 Python directly (no ROS)."""

from __future__ import annotations

import threading
import time
from typing import Any

from ..robot.state import Go2State
from .backend import Go2Backend


class UnitreeSdk2Backend(Go2Backend):
    """High-level locomotion and state transport for Go2/Go2 EDU."""

    def __init__(
        self,
        network_interface: str,
        *,
        domain_id: int = 0,
        state_topic: str = "rt/sportmodestate",
        low_state_topic: str = "rt/lowstate",
        extended_state: bool = False,
        connect_timeout_s: float = 5.0,
    ) -> None:
        self.network_interface = network_interface
        self.domain_id = domain_id
        self.state_topic = state_topic
        self.low_state_topic = low_state_topic
        self.extended_state = extended_state
        self.connect_timeout_s = connect_timeout_s
        self._connected = False
        self._lock = threading.Lock()
        self._state_event = threading.Event()
        self._sport_state: Any | None = None
        self._low_state: Any | None = None
        self._state_received_at = 0.0
        self._sport_client: Any | None = None
        self._state_subscriber: Any | None = None
        self._low_state_subscriber: Any | None = None

    @property
    def is_connected(self) -> bool:
        return self._connected

    def connect(self) -> None:
        if self._connected:
            raise RuntimeError("Unitree SDK2 backend is already connected")
        try:
            from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
            from unitree_sdk2py.go2.sport.sport_client import SportClient
            from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_, SportModeState_
        except ImportError as exc:
            raise ImportError(
                "Unitree SDK2 Python is required for real hardware. Install this package with "
                "the 'unitree' extra after installing CycloneDDS."
            ) from exc

        ChannelFactoryInitialize(self.domain_id, self.network_interface)
        client = SportClient()
        client.SetTimeout(self.connect_timeout_s)
        client.Init()

        state_subscriber = ChannelSubscriber(self.state_topic, SportModeState_)
        state_subscriber.Init(self._on_sport_state, 1)
        low_subscriber = None
        if self.extended_state:
            low_subscriber = ChannelSubscriber(self.low_state_topic, LowState_)
            low_subscriber.Init(self._on_low_state, 1)

        self._sport_client = client
        self._state_subscriber = state_subscriber
        self._low_state_subscriber = low_subscriber
        if not self._state_event.wait(self.connect_timeout_s):
            self.disconnect()
            raise TimeoutError(
                f"No Go2 SportModeState received on {self.state_topic!r} via "
                f"{self.network_interface!r} within {self.connect_timeout_s:.1f}s"
            )
        self._connected = True

    def _on_sport_state(self, message: Any) -> None:
        with self._lock:
            self._sport_state = message
            self._state_received_at = time.monotonic()
        self._state_event.set()

    def _on_low_state(self, message: Any) -> None:
        with self._lock:
            self._low_state = message

    def read_state(self) -> Go2State:
        with self._lock:
            sport = self._sport_state
            low = self._low_state
            received = self._state_received_at
        if sport is None:
            raise RuntimeError("No Go2 state has been received")

        velocity = tuple(float(value) for value in sport.velocity)
        imu = sport.imu_state
        rpy = tuple(float(value) for value in imu.rpy)
        stamp = sport.stamp
        source_timestamp = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        foot_force = tuple(float(value) for value in sport.foot_force)
        battery_voltage = float(getattr(low, "power_v", 0.0)) if low is not None else 0.0
        joint_positions = (
            tuple(float(motor.q) for motor in low.motor_state[:12])
            if low is not None
            else (0.0,) * 12
        )
        extra = {
            "mode": int(sport.mode),
            "gait_type": int(sport.gait_type),
            "position": [float(value) for value in sport.position],
            "range_obstacle": [float(value) for value in sport.range_obstacle],
        }
        return Go2State(
            vx=velocity[0],
            vy=velocity[1],
            wz=float(sport.yaw_speed),
            roll=rpy[0],
            pitch=rpy[1],
            yaw=rpy[2],
            linear_acceleration=tuple(float(value) for value in imu.accelerometer),
            angular_velocity=tuple(float(value) for value in imu.gyroscope),
            body_height=float(sport.body_height),
            foot_contacts=foot_force,
            battery_voltage=battery_voltage,
            joint_positions=joint_positions,
            source_timestamp_s=source_timestamp,
            received_monotonic_s=received,
            extra=extra,
        )

    def send_velocity(self, vx: float, vy: float, wz: float) -> None:
        if self._sport_client is None:
            raise RuntimeError("Unitree SDK2 backend is not connected")
        result = self._sport_client.Move(float(vx), float(vy), float(wz))
        if isinstance(result, int) and result != 0:
            raise RuntimeError(f"SportClient.Move failed with code {result}")

    def stop(self) -> None:
        client = self._sport_client
        if client is None:
            return
        errors: list[Exception] = []
        try:
            client.Move(0.0, 0.0, 0.0)
        except Exception as exc:  # Make a second, independent stop attempt.
            errors.append(exc)
        try:
            result = client.StopMove()
            if isinstance(result, int) and result != 0:
                errors.append(RuntimeError(f"SportClient.StopMove failed with code {result}"))
        except Exception as exc:
            errors.append(exc)
        if len(errors) == 2:
            raise RuntimeError("Both Go2 stop commands failed") from errors[-1]

    def disconnect(self) -> None:
        try:
            self.stop()
        finally:
            for subscriber in (self._state_subscriber, self._low_state_subscriber):
                if subscriber is not None:
                    subscriber.Close()
            self._state_subscriber = None
            self._low_state_subscriber = None
            self._sport_client = None
            self._connected = False
            self._state_event.clear()
