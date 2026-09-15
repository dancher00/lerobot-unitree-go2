from lerobot_robot_unitree_go2.teleoperators import (
    UnitreeGo2KeyboardTeleop,
    UnitreeGo2KeyboardTeleopConfig,
)


class _Listener:
    def is_alive(self) -> bool:
        return True


class _Key:
    space = object()


class _Keyboard:
    Key = _Key


def connected_keyboard() -> UnitreeGo2KeyboardTeleop:
    teleop = UnitreeGo2KeyboardTeleop(UnitreeGo2KeyboardTeleopConfig())
    teleop._listener = _Listener()
    teleop._keyboard = _Keyboard()
    return teleop


def test_ros_keyboard_motion_bindings() -> None:
    teleop = connected_keyboard()

    teleop._on_press("i")
    assert teleop.get_action() == {"base.vx": 0.5, "base.vy": 0.0, "base.wz": 0.0}
    teleop._on_release("i")

    teleop._on_press("u")
    assert teleop.get_action() == {"base.vx": 0.5, "base.vy": 0.0, "base.wz": 0.8}
    teleop._on_release("u")

    teleop._on_press("J")
    assert teleop.get_action() == {"base.vx": 0.0, "base.vy": 0.3, "base.wz": 0.0}


def test_ros_keyboard_stop_and_estop() -> None:
    teleop = connected_keyboard()
    zero = {"base.vx": 0.0, "base.vy": 0.0, "base.wz": 0.0}

    teleop._on_press("i")
    teleop._on_press("K")
    assert teleop.get_action() == zero
    teleop._on_release("K")

    teleop._on_press("x")
    teleop._on_release("x")
    assert teleop.get_action() == zero
    teleop._on_press("v")
    assert teleop.get_action()["base.vx"] == 0.5
