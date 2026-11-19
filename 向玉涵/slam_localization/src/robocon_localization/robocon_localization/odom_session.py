"""Allocate one durable odometry epoch for each managed frontend process."""
import fcntl
import json
import os
from pathlib import Path


def allocate_odom_epoch(path: Path) -> int:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.with_suffix(path.suffix + '.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        previous = json.loads(path.read_text())['odom_epoch'] if path.exists() else 0
        if type(previous) is not int or previous < 0 or previous >= 2**63 - 1:
            raise ValueError('Invalid persisted odometry epoch')
        epoch = previous + 1
        temporary = path.with_suffix(path.suffix + '.tmp')
        with temporary.open('w') as stream:
            json.dump({'odom_epoch': epoch}, stream)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        directory = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
        return epoch
