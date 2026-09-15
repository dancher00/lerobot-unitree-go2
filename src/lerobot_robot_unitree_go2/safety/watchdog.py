"""Independent zero-command watchdog."""

from __future__ import annotations

import logging
import threading
import time
from collections.abc import Callable

logger = logging.getLogger(__name__)


class CommandWatchdog:
    """Call a stop function once when command updates become stale."""

    def __init__(self, timeout_s: float, stop: Callable[[], None]) -> None:
        if timeout_s <= 0:
            raise ValueError("timeout_s must be positive")
        self.timeout_s = timeout_s
        self._stop_callback = stop
        self._lock = threading.Lock()
        self._shutdown = threading.Event()
        self._thread: threading.Thread | None = None
        self._last_feed = 0.0
        self._armed = False
        self._tripped = False

    @property
    def tripped(self) -> bool:
        with self._lock:
            return self._tripped

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._shutdown.clear()
        self._thread = threading.Thread(target=self._run, name="go2-command-watchdog", daemon=True)
        self._thread.start()

    def feed(self) -> None:
        with self._lock:
            self._last_feed = time.monotonic()
            self._armed = True
            self._tripped = False

    def disarm(self) -> None:
        with self._lock:
            self._armed = False

    def _run(self) -> None:
        period = min(0.05, self.timeout_s / 4.0)
        while not self._shutdown.wait(period):
            should_stop = False
            with self._lock:
                if (
                    self._armed
                    and not self._tripped
                    and time.monotonic() - self._last_feed > self.timeout_s
                ):
                    self._tripped = True
                    self._armed = False
                    should_stop = True
            if should_stop:
                logger.error("Go2 command watchdog timed out; sending zero velocity")
                try:
                    self._stop_callback()
                except Exception:
                    logger.exception("Go2 watchdog stop command failed")

    def close(self) -> None:
        self.disarm()
        self._shutdown.set()
        if self._thread is not None:
            self._thread.join(timeout=max(1.0, self.timeout_s * 2.0))
        self._thread = None

    def __enter__(self) -> CommandWatchdog:
        self.start()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
