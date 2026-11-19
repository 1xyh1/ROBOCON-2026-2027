"""Validate MID-360 scan intervals against buffered IMU measurement times."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import json
import time

import numpy as np
import rclpy
from livox_ros_driver2.msg import CustomMsg
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from robocon_core.time_alignment import ImuScanCoverage
from robocon_core.ros_runtime import spin_node
from robocon_core.mid360_protocol import clock_evidence_reasons
from robocon_interfaces.msg import TimeAlignmentStatus


@dataclass
class PendingScan:
    start_s: float
    end_s: float
    received_monotonic: float
    frame_number: int
    reasons: list[str]


class Mid360StampMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__("mid360_stamp_monitor")
        self.declare_parameter("lidar_topic", "/livox/lidar")
        self.declare_parameter("imu_topic", "/livox/imu")
        self.declare_parameter("ptp_verified", False)  # legacy, cannot bypass online evidence
        self.declare_parameter("ptp_utc_offset_s", 37)
        self.declare_parameter("warmup_frames", 10)
        self.declare_parameter("maximum_age_s", 0.20)
        self.declare_parameter("maximum_future_s", 0.02)
        self.declare_parameter("maximum_lidar_imu_gap_s", 0.03)
        self.declare_parameter("imu_arrival_grace_s", 0.05)
        self.declare_parameter("maximum_scan_span_s", 0.20)
        self.declare_parameter("imu_queue_depth", 256)

        self.status_publisher = self.create_publisher(
            TimeAlignmentStatus, "/time_alignment/status", 10
        )
        self.create_subscription(
            Imu,
            str(self.get_parameter("imu_topic").value),
            self._on_imu,
            QoSProfile(depth=int(self.get_parameter("imu_queue_depth").value),
                       reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.create_subscription(
            CustomMsg,
            str(self.get_parameter("lidar_topic").value),
            self._on_lidar,
            qos_profile_sensor_data,
        )
        self._imu = ImuScanCoverage()
        self._pending: deque[PendingScan] = deque()
        self._last_lidar_s: float | None = None
        self._ages: deque[float] = deque(maxlen=200)
        self._frame_count = 0
        self._reset_count = 0
        self._driver_clock = self._device_clock = None
        self._driver_received = self._last_lidar_received = None
        self._driver_epoch = self._device_powerup = None
        self._restart_required = False
        self._evidence_was_valid = False
        self._last_status_received = time.monotonic()
        self.create_subscription(String, "/livox/clock_packets", self._on_packet_clock, 10)
        self.create_subscription(String, "/livox/clock_device", self._on_device_clock, 10)
        self.create_timer(0.01, self._drain_pending)
        self.create_timer(0.1, self._heartbeat)

    def _reset_epoch(self):
        self._imu.times.clear()
        self._pending.clear()
        self._ages.clear()
        self._last_lidar_s = None
        self._frame_count = 0
        self._reset_count += 1

    def _on_packet_clock(self, message):
        try:
            fields = json.loads(message.data)
            clock_evidence_reasons(fields, None, self.get_clock().now().nanoseconds,
                                   int(self.get_parameter("ptp_utc_offset_s").value))
            epoch = (fields["session"], fields["epoch"])
        except (ValueError, KeyError, TypeError, AttributeError):
            self._driver_clock = None
            self._heartbeat()
            return
        if self._driver_epoch is not None and epoch != self._driver_epoch:
            self._restart_required = True
            self._reset_epoch()
        self._driver_epoch = epoch
        self._driver_clock = fields
        self._driver_received = time.monotonic()

    def _on_device_clock(self, message):
        try:
            fields = json.loads(message.data)
            clock_evidence_reasons(None, fields, self.get_clock().now().nanoseconds,
                                   int(self.get_parameter("ptp_utc_offset_s").value))
            if fields.get("valid", False):
                powerup = fields["powerup_count"]
                if self._device_powerup is not None and powerup != self._device_powerup:
                    self._restart_required = True
                    self._reset_epoch()
                self._device_powerup = powerup
        except (ValueError, KeyError, TypeError, AttributeError):
            self._device_clock = None
            self._heartbeat()
            return
        self._device_clock = fields

    def _clock_reasons(self):
        driver = self._driver_clock
        if self._driver_received is None or time.monotonic() - self._driver_received > 0.5:
            driver = None
        reasons = clock_evidence_reasons(driver, self._device_clock,
            self.get_clock().now().nanoseconds, int(self.get_parameter("ptp_utc_offset_s").value))
        if self._restart_required:
            reasons.append("clock_epoch_changed_restart_lio")
        return list(dict.fromkeys(reasons))

    def _heartbeat(self):
        reasons = self._clock_reasons()
        if self._last_lidar_received is None or time.monotonic() - self._last_lidar_received > 0.3:
            reasons.append("lidar_stream_missing")
        if reasons:
            if self._evidence_was_valid:
                self._reset_epoch()
            self._evidence_was_valid = False
            result = TimeAlignmentStatus()
            result.header.stamp = self.get_clock().now().to_msg()
            result.sensor, result.valid = "mid360_ptp_clock", False
            result.reset_count = self._reset_count
            result.detail = ",".join(reasons)
            self.status_publisher.publish(result)
        else:
            self._evidence_was_valid = True

    def _on_imu(self, message: Imu) -> None:
        self._imu.add(self._header_seconds(message.header))
        self._drain_pending()

    def _on_lidar(self, message: CustomMsg) -> None:
        self._last_lidar_received = time.monotonic()
        self._frame_count += 1
        lidar_s = float(message.timebase) * 1e-9
        header_s = self._header_seconds(message.header)
        reasons: list[str] = []

        if self._last_lidar_s is not None and lidar_s <= self._last_lidar_s:
            self._reset_epoch()
            self._restart_required = True
            reasons.append("lidar_stamp_not_monotonic")
        self._last_lidar_s = lidar_s
        if abs(header_s - lidar_s) > 1e-6:
            reasons.append("header_timebase_mismatch")
        start_s = end_s = lidar_s
        if int(message.point_num) != len(message.points) or not message.points:
            reasons.append("point_count")
        else:
            offsets = np.fromiter((int(point.offset_time) for point in message.points), dtype=np.int64)
            maximum_span_ns = float(self.get_parameter("maximum_scan_span_s").value) * 1e9
            # MID-360 packets can interleave scan lines: valid per-point times
            # are not necessarily ordered in the CustomMsg point array.
            if np.min(offsets) < 0 or np.max(offsets) > maximum_span_ns:
                reasons.append("point_offset_span")
            else:
                start_s += int(np.min(offsets)) * 1e-9
                end_s += int(np.max(offsets)) * 1e-9
        if self._frame_count < int(self.get_parameter("warmup_frames").value):
            reasons.append("warming_up")

        self._pending.append(PendingScan(start_s, end_s, time.monotonic(), self._frame_count, reasons))
        self._drain_pending()

    def _drain_pending(self) -> None:
        while self._pending:
            scan = self._pending[0]
            coverage = self._imu.check(
                scan.start_s, scan.end_s,
                float(self.get_parameter("maximum_lidar_imu_gap_s").value),
            )
            waiting = time.monotonic() - scan.received_monotonic
            if not scan.reasons and coverage != "ok" and waiting < float(self.get_parameter("imu_arrival_grace_s").value):
                # Callbacks may arrive in either order. Wait for the sample
                # bracketing the last point before reporting missing IMU data.
                return
            self._pending.popleft()
            reasons = list(scan.reasons)
            if coverage != "ok":
                reasons.append(coverage)
            reasons.extend(self._clock_reasons())
            host_s = self.get_clock().now().nanoseconds * 1e-9
            age_s = host_s - scan.end_s
            self._ages.append(age_s)
            if age_s > float(self.get_parameter("maximum_age_s").value):
                reasons.append("lidar_stamp_stale")
            if age_s < -float(self.get_parameter("maximum_future_s").value):
                reasons.append("lidar_stamp_in_future")
            ages = np.asarray(self._ages)
            result = TimeAlignmentStatus()
            result.header.stamp = self.get_clock().now().to_msg()
            result.sensor = "mid360_ptp_clock"
            result.valid = not reasons
            result.sample_count = scan.frame_number
            result.scale = 1.0
            result.offset_s = 0.0
            result.skew_ppm = 0.0
            result.delay_p50_s = float(np.quantile(ages, 0.50))
            result.delay_p95_s = float(np.quantile(ages, 0.95))
            result.jitter_s = float(np.quantile(ages, 0.95) - np.quantile(ages, 0.05))
            result.reset_count = self._reset_count
            result.detail = "ok:imu_covers_scan;packet_ptp;device_sync_recent" if not reasons else ",".join(reasons)
            self.status_publisher.publish(result)

    @staticmethod
    def _header_seconds(header) -> float:
        return float(header.stamp.sec) + float(header.stamp.nanosec) * 1e-9


def main(args: list[str] | None = None) -> None:
    spin_node(Mid360StampMonitorNode, args)


if __name__ == "__main__":
    main()
