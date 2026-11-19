"""Sliding affine clock alignment for sensors without a PTP-capable clock."""

from __future__ import annotations

from collections import deque
from bisect import bisect_left, bisect_right, insort
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ClockFit:
    valid: bool
    sample_count: int
    scale: float
    offset_s: float
    skew_ppm: float
    delay_p50_s: float
    delay_p95_s: float
    jitter_s: float
    reset_count: int
    reason: str


class SlidingAffineClockMapper:
    """Map a monotonic sensor clock onto host time using its arrival-time envelope.

    One-way USB delivery cannot reveal the absolute transport delay. The lower
    arrival-time envelope removes scheduling spikes and estimates clock skew;
    ``fixed_offset_s`` is reserved for a separately measured residual offset.
    """

    def __init__(
        self,
        *,
        window_samples: int = 300,
        min_samples: int = 30,
        min_span_s: float = 0.8,
        envelope_quantile: float = 0.10,
        max_abs_skew_ppm: float = 1500.0,
        max_jitter_s: float = 0.020,
        fixed_offset_s: float = 0.0,
    ) -> None:
        if window_samples < min_samples or min_samples < 3:
            raise ValueError("window_samples must be >= min_samples >= 3")
        if not 0.0 < envelope_quantile < 0.5:
            raise ValueError("envelope_quantile must be between 0 and 0.5")
        self.samples: deque[tuple[float, float]] = deque(maxlen=window_samples)
        self.min_samples = min_samples
        self.min_span_s = min_span_s
        self.envelope_quantile = envelope_quantile
        self.max_abs_skew_ppm = max_abs_skew_ppm
        self.max_jitter_s = max_jitter_s
        self.fixed_offset_s = fixed_offset_s
        self.reset_count = 0
        self._last_sensor_s: float | None = None
        self._fit = ClockFit(
            False, 0, 1.0, 0.0, 0.0, float("nan"), float("nan"),
            float("nan"), 0, "insufficient_samples"
        )

    @property
    def fit(self) -> ClockFit:
        return self._fit

    def reset(self) -> None:
        """Discard an old clock domain or acquisition session."""
        self.samples.clear()
        self._last_sensor_s = None
        self.reset_count += 1
        self._fit = self._invalid(0, "clock_reset")

    def update(self, sensor_time_s: float, host_receive_time_s: float) -> ClockFit:
        if not np.isfinite(sensor_time_s) or not np.isfinite(host_receive_time_s):
            raise ValueError("timestamps must be finite")
        if self._last_sensor_s is not None and sensor_time_s <= self._last_sensor_s:
            self.samples.clear()
            self.reset_count += 1
        self._last_sensor_s = sensor_time_s
        self.samples.append((sensor_time_s, host_receive_time_s))
        self._fit = self._estimate()
        return self._fit

    def map_to_host(self, sensor_time_s: float) -> float:
        if not self._fit.valid:
            raise RuntimeError(f"clock fit is not valid: {self._fit.reason}")
        return (
            self._fit.scale * sensor_time_s
            + self._fit.offset_s
            + self.fixed_offset_s
        )

    def _estimate(self) -> ClockFit:
        count = len(self.samples)
        if count < self.min_samples:
            return self._invalid(count, "insufficient_samples")
        data = np.asarray(self.samples, dtype=np.float64)
        sensor = data[:, 0]
        host = data[:, 1]
        span = float(sensor[-1] - sensor[0])
        if span < self.min_span_s:
            return self._invalid(count, "insufficient_time_span")

        sensor_center = float(np.mean(sensor))
        host_center = float(np.mean(host))
        denominator = float(np.sum((sensor - sensor_center) ** 2))
        if denominator <= 1e-12:
            return self._invalid(count, "degenerate_sensor_clock")
        initial_scale = float(
            np.sum((sensor - sensor_center) * (host - host_center)) / denominator
        )
        offsets = host - initial_scale * sensor
        cutoff = float(np.quantile(offsets, 0.35))
        lower = offsets <= cutoff
        if int(np.count_nonzero(lower)) < 3:
            return self._invalid(count, "insufficient_envelope_samples")

        x = sensor[lower]
        y = host[lower]
        x_center = float(np.mean(x))
        y_center = float(np.mean(y))
        denominator = float(np.sum((x - x_center) ** 2))
        if denominator <= 1e-12:
            return self._invalid(count, "degenerate_envelope")
        scale = float(np.sum((x - x_center) * (y - y_center)) / denominator)
        offset_candidates = host - scale * sensor
        offset = float(np.quantile(offset_candidates, self.envelope_quantile))
        delays = host - (scale * sensor + offset)
        delay_p50 = float(np.quantile(delays, 0.50))
        delay_p95 = float(np.quantile(delays, 0.95))
        delay_p05 = float(np.quantile(delays, 0.05))
        jitter = delay_p95 - delay_p05
        skew_ppm = (scale - 1.0) * 1e6

        reason = "ok"
        valid = True
        if abs(skew_ppm) > self.max_abs_skew_ppm:
            valid, reason = False, "clock_skew_out_of_range"
        elif jitter > self.max_jitter_s:
            valid, reason = False, "arrival_jitter_out_of_range"
        return ClockFit(
            valid=valid,
            sample_count=count,
            scale=scale,
            offset_s=offset,
            skew_ppm=skew_ppm,
            delay_p50_s=delay_p50,
            delay_p95_s=delay_p95,
            jitter_s=jitter,
            reset_count=self.reset_count,
            reason=reason,
        )

    def _invalid(self, count: int, reason: str) -> ClockFit:
        previous = self._fit
        return ClockFit(
            valid=False,
            sample_count=count,
            scale=previous.scale,
            offset_s=previous.offset_s,
            skew_ppm=previous.skew_ppm,
            delay_p50_s=previous.delay_p50_s,
            delay_p95_s=previous.delay_p95_s,
            jitter_s=previous.jitter_s,
            reset_count=self.reset_count,
            reason=reason,
        )


@dataclass(frozen=True)
class FrameClockResult:
    valid: bool
    color_time_s: float | None
    depth_time_s: float | None
    reason: str
    fit: ClockFit


class RealSenseFrameClock:
    """Separate SDK host timestamps from an uncalibrated USB arrival fit.

    ``ClockFit.valid`` only describes a numerical arrival fit. It is never by
    itself evidence that a hardware clock is synchronized to capture time.
    """

    def __init__(
        self,
        mapper: SlidingAffineClockMapper,
        *,
        calibration_verified: bool = False,
        maximum_age_s: float = 0.25,
        maximum_future_s: float = 0.02,
        maximum_color_depth_delta_s: float = 0.02,
    ) -> None:
        self.mapper = mapper
        self.calibration_verified = calibration_verified
        self.maximum_age_s = maximum_age_s
        self.maximum_future_s = maximum_future_s
        self.maximum_color_depth_delta_s = maximum_color_depth_delta_s
        self._domain: str | None = None
        self._last_times: tuple[float, float] | None = None
        self._ages: deque[float] = deque(maxlen=300)
        self._sample_count = 0

    def update(
        self, color_s: float, depth_s: float, color_domain: str,
        depth_domain: str, host_receive_s: float,
    ) -> FrameClockResult:
        fit = self.mapper.fit
        if not np.isfinite([color_s, depth_s, host_receive_s]).all():
            return FrameClockResult(False, None, None, "non_finite_timestamp", fit)
        if color_domain not in {"hardware_clock", "system_time", "global_time"} or depth_domain != color_domain:
            return FrameClockResult(False, None, None, "clock_domain_mismatch", fit)
        if self._domain != color_domain:
            if self._domain is not None:
                self.mapper.reset()
            self._domain = color_domain
            self._last_times = None
            self._ages.clear()
            self._sample_count = 0
        previous = self._last_times
        self._last_times = (color_s, depth_s)
        self._sample_count += 1
        if previous is not None and (color_s <= previous[0] or depth_s <= previous[1]):
            self.mapper.reset()
            self._ages.clear()
            return FrameClockResult(False, None, None, "frame_clock_not_monotonic", self.mapper.fit)
        if abs(depth_s - color_s) > self.maximum_color_depth_delta_s:
            return FrameClockResult(False, None, None, "color_depth_delta", fit)

        if color_domain == "hardware_clock":
            fit = self.mapper.update(color_s, host_receive_s)
            if not fit.valid:
                return FrameClockResult(False, None, None, fit.reason, fit)
            color_time = self.mapper.map_to_host(color_s)
            depth_time = self.mapper.map_to_host(depth_s)
            if not self.calibration_verified:
                return FrameClockResult(False, color_time, depth_time, "hardware_offset_not_calibrated", fit)
        else:
            # SDK global_time/system_time already uses the host epoch. Never
            # replace it with a fit to USB/ROS callback arrival timestamps.
            color_time = color_s + self.mapper.fixed_offset_s
            depth_time = depth_s + self.mapper.fixed_offset_s
            self._ages.append(host_receive_s - color_time)
            ages = np.asarray(self._ages)
            fit = ClockFit(
                True, self._sample_count, 1.0, 0.0, 0.0,
                float(np.quantile(ages, 0.5)), float(np.quantile(ages, 0.95)),
                float(np.quantile(ages, 0.95) - np.quantile(ages, 0.05)),
                self.mapper.reset_count, "sdk_" + color_domain,
            )
            # librealsense SYSTEM_TIME can be the USB arrival timestamp when
            # hardware metadata is unavailable; epoch agreement is insufficient.
            if color_domain == "system_time" and not self.calibration_verified:
                return FrameClockResult(False, color_time, depth_time, "system_time_capture_not_verified", fit)

        ages = [host_receive_s - color_time, host_receive_s - depth_time]
        reason = "ok:" + color_domain
        if max(ages) > self.maximum_age_s:
            reason = "frame_stamp_stale"
        elif min(ages) < -self.maximum_future_s:
            reason = "frame_stamp_in_future"
        return FrameClockResult(reason.startswith("ok:"), color_time, depth_time, reason, fit)


class ImuScanCoverage:
    """Bounded sorted IMU times, tolerant of callback arrival reordering.

    Access belongs to the ROS single-threaded executor. Add synchronization
    here before moving its callbacks to separate concurrent callback groups.
    """

    def __init__(self, max_samples: int = 2000) -> None:
        self.max_samples = max_samples
        self.times: list[float] = []

    def add(self, timestamp_s: float) -> None:
        if not np.isfinite(timestamp_s):
            return
        index = bisect_left(self.times, timestamp_s)
        if index < len(self.times) and self.times[index] == timestamp_s:
            return
        insort(self.times, timestamp_s)
        if len(self.times) > self.max_samples:
            del self.times[:len(self.times) - self.max_samples]

    def check(self, scan_start_s: float, scan_end_s: float, maximum_gap_s: float) -> str:
        if not np.isfinite([scan_start_s, scan_end_s, maximum_gap_s]).all() or maximum_gap_s <= 0:
            return "imu_scan_range_invalid"
        if scan_end_s < scan_start_s:
            return "imu_scan_range_inverted"
        if not self.times:
            return "imu_missing"
        left = bisect_right(self.times, scan_start_s) - 1
        right = bisect_left(self.times, scan_end_s)
        if left < 0:
            return "imu_scan_start_uncovered"
        if right >= len(self.times):
            return "imu_scan_end_uncovered"
        # Require a measurement at/before the first point and at/after the last
        # point, plus no missing IMU interval within the entire scan.
        if any(b - a > maximum_gap_s for a, b in zip(self.times[left:right], self.times[left + 1:right + 1])):
            return "imu_scan_gap"
        return "ok"
