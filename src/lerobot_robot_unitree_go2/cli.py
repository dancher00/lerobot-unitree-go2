"""Small alias for the standard LeRobot recorder."""


def record_main() -> None:
    """Run unmodified ``lerobot-record`` with third-party plugin discovery."""
    from lerobot.scripts.lerobot_record import main

    main()
