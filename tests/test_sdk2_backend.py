from types import SimpleNamespace

from lerobot_robot_unitree_go2.unitree import UnitreeSdk2Backend


def test_sdk_state_mapping_without_sdk_install() -> None:
    backend = UnitreeSdk2Backend("eth0", extended_state=True)
    backend._on_low_state(
        SimpleNamespace(power_v=27.4, motor_state=[SimpleNamespace(q=i / 10) for i in range(12)])
    )
    backend._on_sport_state(
        SimpleNamespace(
            velocity=[0.2, -0.1, 0.0],
            yaw_speed=0.3,
            imu_state=SimpleNamespace(
                rpy=[0.01, 0.02, 0.03],
                accelerometer=[1.0, 2.0, 3.0],
                gyroscope=[0.1, 0.2, 0.3],
            ),
            stamp=SimpleNamespace(sec=12, nanosec=500_000_000),
            foot_force=[1, 2, 3, 4],
            body_height=0.31,
            mode=1,
            gait_type=2,
            position=[3.0, 4.0, 5.0],
            range_obstacle=[6.0, 7.0, 8.0, 9.0],
        )
    )
    state = backend.read_state()
    assert (state.vx, state.vy, state.wz) == (0.2, -0.1, 0.3)
    assert (state.roll, state.pitch, state.yaw) == (0.01, 0.02, 0.03)
    assert state.source_timestamp_s == 12.5
    assert state.battery_voltage == 27.4
    assert len(state.joint_positions) == 12


def test_sdk_stop_uses_zero_move_and_stop_move() -> None:
    calls: list[tuple] = []

    class Client:
        def Move(self, vx: float, vy: float, wz: float) -> int:
            calls.append(("Move", vx, vy, wz))
            return 0

        def StopMove(self) -> int:
            calls.append(("StopMove",))
            return 0

    backend = UnitreeSdk2Backend("eth0")
    backend._sport_client = Client()
    backend.stop()
    assert calls == [("Move", 0.0, 0.0, 0.0), ("StopMove",)]
