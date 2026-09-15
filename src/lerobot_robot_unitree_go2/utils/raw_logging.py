"""Optional JSONL diagnostics, separate from the LeRobot dataset."""

import json
import threading
from pathlib import Path
from typing import Any, TextIO


class RawJsonlLogger:
    def __init__(self, path: Path | None) -> None:
        self.path = path
        self._file: TextIO | None = None
        self._lock = threading.Lock()

    def open(self) -> None:
        if self.path is None or self._file is not None:
            return
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self.path.open("a", encoding="utf-8")

    def write(self, record: dict[str, Any]) -> None:
        if self._file is None:
            return
        with self._lock:
            self._file.write(json.dumps(record, separators=(",", ":"), default=str) + "\n")
            self._file.flush()

    def close(self) -> None:
        if self._file is not None:
            self._file.close()
            self._file = None
