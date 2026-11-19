"""One-shot colour transition detector for a possible official start signal."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import time
import math

import cv2
import numpy as np


class GateState(str, Enum):
    DISARMED = "disarmed"
    WAITING_FOR_BASELINE = "waiting_for_baseline"
    WAITING_FOR_START = "waiting_for_start"
    STARTED = "started"


@dataclass(frozen=True)
class StartGateConfig:
    roi_xywh_norm: tuple[float, float, float, float] = (0.35, 0.25, 0.30, 0.50)
    waiting_color: str = "red"
    start_color: str = "green"
    min_color_fraction: float = 0.35
    baseline_frames: int = 3
    start_frames: int = 3
    max_frame_gap_s: float = 0.25


@dataclass(frozen=True)
class StartSignalEvent:
    sequence: int
    monotonic_time_s: float
    source: str
    color: str
    color_fraction: float


class ColorStartGate:
    """Require a visible waiting state followed by a stable start colour."""

    _RANGES = {
        "red": [((0, 90, 60), (12, 255, 255)), ((168, 90, 60), (179, 255, 255))],
        "green": [((38, 70, 50), (90, 255, 255))],
        "blue": [((95, 80, 45), (140, 255, 255))],
        "yellow": [((18, 80, 70), (37, 255, 255))],
    }

    def __init__(self, config: StartGateConfig | None = None) -> None:
        self.config = config or StartGateConfig()
        x, y, width, height = self.config.roi_xywh_norm
        if (not all(math.isfinite(value) for value in (x, y, width, height))
                or x < 0 or y < 0 or width <= 0 or height <= 0
                or x + width > 1 or y + height > 1):
            raise ValueError("ROI must have positive dimensions and stay inside the normalized image")
        if self.config.waiting_color not in self._RANGES or self.config.start_color not in self._RANGES:
            raise ValueError("unsupported waiting/start colour")
        if self.config.waiting_color == self.config.start_color:
            raise ValueError("waiting and start colours must differ")
        if (self.config.baseline_frames < 1 or self.config.start_frames < 1
                or not math.isfinite(self.config.max_frame_gap_s) or self.config.max_frame_gap_s <= 0):
            raise ValueError("frame counts and max frame gap must be positive")
        if not 0 < self.config.min_color_fraction <= 1:
            raise ValueError("min_color_fraction must be in (0, 1]")
        self.state = GateState.DISARMED
        self._baseline_count = 0
        self._start_count = 0
        self._sequence = 0
        self._last_timestamp_s: float | None = None

    def arm(self) -> None:
        self.state = GateState.WAITING_FOR_BASELINE
        self._baseline_count = 0
        self._start_count = 0
        self._last_timestamp_s = None

    def disarm(self) -> None:
        self.state = GateState.DISARMED

    def update(self, image_bgr: np.ndarray, *, timestamp_s: float | None = None) -> StartSignalEvent | None:
        if self.state in (GateState.DISARMED, GateState.STARTED):
            return None
        timestamp = time.monotonic() if timestamp_s is None else timestamp_s
        if not math.isfinite(timestamp):
            return None
        if self._last_timestamp_s is not None:
            delta = timestamp - self._last_timestamp_s
            if delta <= 0:
                # Duplicate/reordered images never count as new observations.
                self._start_count = 0
                return None
            if delta > self.config.max_frame_gap_s:
                self.state = GateState.WAITING_FOR_BASELINE
                self._baseline_count = self._start_count = 0
        self._last_timestamp_s = timestamp
        fractions = self.measure(image_bgr)
        waiting = fractions[self.config.waiting_color] >= self.config.min_color_fraction
        started = fractions[self.config.start_color] >= self.config.min_color_fraction
        if self.state == GateState.WAITING_FOR_BASELINE:
            self._baseline_count = self._baseline_count + 1 if waiting else 0
            if self._baseline_count >= self.config.baseline_frames:
                self.state = GateState.WAITING_FOR_START
            return None
        self._start_count = self._start_count + 1 if started and not waiting else 0
        if self._start_count < self.config.start_frames:
            return None
        self._sequence += 1
        self.state = GateState.STARTED
        return StartSignalEvent(
            sequence=self._sequence,
            monotonic_time_s=timestamp,
            source="camera_color_transition",
            color=self.config.start_color,
            color_fraction=fractions[self.config.start_color],
        )

    def measure(self, image_bgr: np.ndarray) -> dict[str, float]:
        if image_bgr.ndim != 3 or image_bgr.shape[2] != 3:
            raise ValueError("image_bgr must have shape HxWx3")
        h, w = image_bgr.shape[:2]
        nx, ny, nw, nh = self.config.roi_xywh_norm
        x0, y0 = int(nx * w), int(ny * h)
        x1, y1 = int((nx + nw) * w), int((ny + nh) * h)
        roi = image_bgr[max(0, y0) : min(h, y1), max(0, x0) : min(w, x1)]
        if roi.size == 0:
            raise ValueError("configured ROI is empty")
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        fractions: dict[str, float] = {}
        for color, ranges in self._RANGES.items():
            mask = np.zeros(hsv.shape[:2], np.uint8)
            for lower, upper in ranges:
                mask |= cv2.inRange(hsv, np.array(lower), np.array(upper))
            fractions[color] = float(np.count_nonzero(mask) / mask.size)
        return fractions
