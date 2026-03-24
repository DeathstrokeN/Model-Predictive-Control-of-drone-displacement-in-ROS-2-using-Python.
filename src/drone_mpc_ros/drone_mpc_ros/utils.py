"""Utility helpers for the ROS 2 drone MPC project."""

from pathlib import Path
import csv


def ensure_log_directory() -> Path:
    """Create the top-level logs directory if it does not exist."""
    log_dir = Path.cwd() / 'logs'
    log_dir.mkdir(parents=True, exist_ok=True)
    return log_dir


class CsvLogger:
    """Simple CSV logger used by the controller node.

    We keep it intentionally lightweight so the project stays easy to read.
    """

    def __init__(self, filename: str, header: list[str]):
        self.path = ensure_log_directory() / filename
        self.file = open(self.path, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(header)
        self.file.flush()

    def write_row(self, row: list[float]) -> None:
        self.writer.writerow(row)
        self.file.flush()

    def close(self) -> None:
        try:
            self.file.close()
        except Exception:
            pass
