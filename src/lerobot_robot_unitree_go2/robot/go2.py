"""LeRobot Robot implementation for Unitree Go2 / Go2 EDU."""

from __future__ import annotations

import logging
import time
from functools import cached_property

import numpy as np
from lerobot.cameras import make_cameras_from_configs
from lerobot.lerobot_types import RobotAction, RobotObservation
from lerobot.robots import Robot
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from ..safety.watchdog import CommandWatchdog
from ..unitree.backend import Go2Backend, MockGo2Backend
from ..unitree.sdk2_backend import UnitreeSdk2Backend
from ..utils.raw_logging import RawJsonlLogger
from .configuration_go2 import UnitreeGo2Config
from .mock_camera import GeneratedCamera

logger = logging.getLogger(__name__)

ACTION_KEYS = ("base.vx", "base.vy", "base.wz")


class UnitreeGo2(Robot):
    """Go2 adapter with velocity clipping, stale-state checks, and stop-on-exit."""

    config_class = UnitreeGo2Config
    name = "unitree_go2"

    def __init__(self, config: UnitreeGo2Config, backend: Go2Backend | None = None) -> None:
        super().__init__(config)
        self.config = config
        self.backend = backend or self._make_backend()
        self.cameras = make_cameras_from_configs(config.cameras)
        if config.mock and config.mock_camera and "front" not in self.cameras:
            self.cameras["front"] = GeneratedCamera(
                config.mock_camera_width, config.mock_camera_height
            )
        self._watchdog = CommandWatchdog(config.watchdog_timeout_s, self._watchdog_stop)
        self._raw_logger = RawJsonlLogger(config.raw_log_path)
        self._last_observation_monotonic: float | None = None

    def _make_backend(self) -> Go2Backend:
        if self.config.mock:
            return MockGo2Backend(self.config.mock_response_rate)
        return UnitreeSdk2Backend(
            self.config.network_interface,
            domain_id=self.config.domain_id,
            state_topic=self.config.state_topic,
            low_state_topic=self.config.low_state_topic,
            extended_state=self.config.extended_state,
            connect_timeout_s=self.config.connect_timeout_s,
        )

    @cached_property
    def observation_features(self) -> dict[str, type | tuple[int, int, int]]:
        state_features = {
            "base.vx": float,
            "base.vy": float,
            "base.wz": float,
            "base.roll": float,
            "base.pitch": float,
            "base.yaw": float,
        }
        if self.config.extended_state:
            state_features.update(
                {
                    **{f"base.linear_acceleration.{axis}": float for axis in "xyz"},
                    **{f"base.angular_velocity.{axis}": float for axis in "xyz"},
                    "base.height": float,
                    **{f"base.foot_contact.{index}": float for index in range(4)},
                    "base.battery_voltage": float,
                    **{f"base.joint.{index}.position": float for index in range(12)},
                }
            )
        camera_features: dict[str, tuple[int, int, int]] = {}
        for name, camera in self.cameras.items():
            if getattr(camera, "use_rgb", True):
                camera_features[name] = (int(camera.height), int(camera.width), 3)
            if getattr(camera, "use_depth", False):
                camera_features[f"{name}_depth"] = (int(camera.height), int(camera.width), 1)
        return {**state_features, **camera_features}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return dict.fromkeys(ACTION_KEYS, float)

    @property
    def is_connected(self) -> bool:
        return self.backend.is_connected and all(
            camera.is_connected for camera in self.cameras.values()
        )

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        """Go2 high-level velocity control requires no adapter calibration."""

    def configure(self) -> None:
        """No persistent robot configuration is changed by this adapter."""

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        del calibrate
        connected_cameras = []
        try:
            self.backend.connect()
            for camera in self.cameras.values():
                camera.connect()
                connected_cameras.append(camera)
            self._raw_logger.open()
            self._watchdog.start()
        except BaseException:
            try:
                self.backend.stop()
            except Exception:
                logger.exception("Failed to stop Go2 while rolling back connect")
            for camera in reversed(connected_cameras):
                try:
                    camera.disconnect()
                except Exception:
                    logger.exception("Failed to disconnect camera while rolling back connect")
            try:
                self.backend.disconnect()
            except Exception:
                logger.exception("Failed to disconnect Go2 backend while rolling back connect")
            raise
        logger.info("Connected to %s", self)

    def _watchdog_stop(self) -> None:
        self.backend.stop()
        self._raw_logger.write({"type": "watchdog_stop", "monotonic_s": time.monotonic()})

    def emergency_stop(self, reason: str = "requested") -> None:
        """Immediately disarm the watchdog and issue redundant SDK stop calls."""
        self._watchdog.disarm()
        self.backend.stop()
        self._raw_logger.write(
            {"type": "emergency_stop", "reason": reason, "monotonic_s": time.monotonic()}
        )

    def _read_camera(self, name: str, camera: object) -> np.ndarray:
        for attempt in range(self.config.camera_reconnect_attempts + 1):
            try:
                return camera.read_latest(max_age_ms=self.config.camera_max_age_ms)  # type: ignore[attr-defined]
            except (TimeoutError, RuntimeError, ConnectionError):
                if (
                    not self.config.camera_reconnect
                    or attempt >= self.config.camera_reconnect_attempts
                ):
                    raise
                logger.warning("Camera %s stalled; reconnecting", name)
                try:
                    camera.disconnect()  # type: ignore[attr-defined]
                except Exception:
                    logger.debug("Camera %s cleanup before reconnect failed", name, exc_info=True)
                camera.connect()  # type: ignore[attr-defined]
        raise AssertionError("unreachable")

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        sample_time = time.monotonic()
        try:
            state = self.backend.read_state()
            state_age = sample_time - state.received_monotonic_s
            if state_age > self.config.state_timeout_s:
                raise TimeoutError(
                    f"Go2 state is stale ({state_age:.3f}s > {self.config.state_timeout_s:.3f}s)"
                )
            observation: RobotObservation = (
                state.extended_observation()
                if self.config.extended_state
                else state.minimal_observation()
            )
            camera_timestamps: dict[str, float | None] = {}
            for name, camera in self.cameras.items():
                if getattr(camera, "use_rgb", True):
                    observation[name] = self._read_camera(name, camera)
                if getattr(camera, "use_depth", False):
                    observation[f"{name}_depth"] = camera.read_latest_depth(
                        max_age_ms=self.config.camera_max_age_ms
                    )
                camera_timestamps[name] = getattr(camera, "latest_timestamp", None)
        except BaseException:
            try:
                self.emergency_stop("observation failure")
            except Exception:
                logger.exception("Stop after observation failure also failed")
            raise

        self._last_observation_monotonic = sample_time
        self._raw_logger.write(
            {
                "type": "observation",
                "sample_monotonic_s": sample_time,
                "camera_monotonic_s": camera_timestamps,
                "state": state.to_dict(),
            }
        )
        return observation

    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        missing = set(ACTION_KEYS).difference(action)
        if missing:
            raise ValueError(f"Go2 action is missing keys: {sorted(missing)}")
        sent: RobotAction = {
            "base.vx": float(np.clip(action["base.vx"], -self.config.max_vx, self.config.max_vx)),
            "base.vy": float(np.clip(action["base.vy"], -self.config.max_vy, self.config.max_vy)),
            "base.wz": float(np.clip(action["base.wz"], -self.config.max_wz, self.config.max_wz)),
        }
        # Current lerobot-record records the processed teleop dict after send_action.
        # Updating it in place keeps the Dataset v3 action exactly equal to the clipped command.
        action.update(sent)
        try:
            self.backend.send_velocity(sent["base.vx"], sent["base.vy"], sent["base.wz"])
            self._watchdog.feed()
        except BaseException:
            try:
                self.emergency_stop("command failure")
            except Exception:
                logger.exception("Stop after command failure also failed")
            raise
        self._raw_logger.write(
            {"type": "action", "monotonic_s": time.monotonic(), "action": dict(sent)}
        )
        return sent

    def disconnect(self) -> None:
        """Stop first, then release cameras and DDS resources; may be called repeatedly."""
        self._watchdog.close()
        try:
            self.backend.stop()
        except Exception:
            logger.exception("Failed to send Go2 zero velocity during disconnect")
        for name, camera in self.cameras.items():
            if camera.is_connected:
                try:
                    camera.disconnect()
                except Exception:
                    logger.exception("Failed to disconnect camera %s", name)
        try:
            self.backend.disconnect()
        finally:
            self._raw_logger.close()
        logger.info("Disconnected from %s", self)
