"""Preserve SDK host times and explicitly gate calibrated hardware clock fits."""

from __future__ import annotations

import copy
from collections import Counter
import json
import math
import time

import rclpy
from message_filters import TimeSynchronizer, Subscriber
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from realsense2_camera_msgs.msg import Metadata
from sensor_msgs.msg import CameraInfo, Image

from robocon_core.time_alignment import ClockFit, RealSenseFrameClock, SlidingAffineClockMapper
from robocon_core.ros_runtime import spin_node
from robocon_interfaces.msg import TimeAlignmentStatus


class D435iTimeAlignNode(Node):
    def __init__(self) -> None:
        super().__init__("d435i_time_align")
        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter(
            "depth_topic", "/camera/camera/aligned_depth_to_color/image_raw"
        )
        self.declare_parameter("metadata_topic", "/camera/camera/color/metadata")
        self.declare_parameter(
            "depth_metadata_topic", "/camera/camera/depth/metadata"
        )
        self.declare_parameter(
            "camera_info_topic", "/camera/camera/color/camera_info"
        )
        self.declare_parameter("output_prefix", "/sensors/d435i")
        self.declare_parameter("window_samples", 300)
        self.declare_parameter("minimum_samples", 30)
        self.declare_parameter("minimum_span_s", 0.8)
        self.declare_parameter("maximum_jitter_s", 0.020)
        self.declare_parameter("maximum_color_depth_delta_s", 0.020)
        self.declare_parameter("fixed_offset_s", 0.0)
        self.declare_parameter("calibration_verified", False)
        self.declare_parameter("maximum_age_s", 0.25)
        self.declare_parameter("maximum_future_s", 0.02)
        self.declare_parameter("frame_queue_depth", 16)
        frame_depth = int(self.get_parameter("frame_queue_depth").value)
        frame_qos = QoSProfile(depth=frame_depth, reliability=ReliabilityPolicy.RELIABLE)
        self._input_counts = Counter()
        self._frame_counts = Counter()
        self._rejection_counts = Counter()
        self._last_pair_monotonic = None

        self.mapper = SlidingAffineClockMapper(
            window_samples=int(self.get_parameter("window_samples").value),
            min_samples=int(self.get_parameter("minimum_samples").value),
            min_span_s=float(self.get_parameter("minimum_span_s").value),
            max_jitter_s=float(self.get_parameter("maximum_jitter_s").value),
            fixed_offset_s=float(self.get_parameter("fixed_offset_s").value),
        )
        self.frame_clock = RealSenseFrameClock(
            self.mapper,
            calibration_verified=bool(self.get_parameter("calibration_verified").value),
            maximum_age_s=float(self.get_parameter("maximum_age_s").value),
            maximum_future_s=float(self.get_parameter("maximum_future_s").value),
            maximum_color_depth_delta_s=float(self.get_parameter("maximum_color_depth_delta_s").value),
        )
        prefix = str(self.get_parameter("output_prefix").value).rstrip("/")
        self.color_publisher = self.create_publisher(
            Image, prefix + "/color/image_raw", frame_qos
        )
        self.depth_publisher = self.create_publisher(
            Image,
            prefix + "/aligned_depth_to_color/image_raw",
            frame_qos,
        )
        self.info_publisher = self.create_publisher(
            CameraInfo, prefix + "/color/camera_info", frame_qos
        )
        self.status_publisher = self.create_publisher(
            TimeAlignmentStatus, "/time_alignment/status", 10
        )
        self.diagnostic_publisher = self.create_publisher(
            DiagnosticArray, prefix + "/diagnostics", 10)
        camera_info = Subscriber(
            self,
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            qos_profile=frame_qos,
        )

        color = Subscriber(
            self,
            Image,
            str(self.get_parameter("color_topic").value),
            qos_profile=frame_qos,
        )
        depth = Subscriber(
            self,
            Image,
            str(self.get_parameter("depth_topic").value),
            qos_profile=frame_qos,
        )
        color_metadata = Subscriber(
            self,
            Metadata,
            str(self.get_parameter("metadata_topic").value),
            qos_profile=frame_qos,
        )
        depth_metadata = Subscriber(
            self,
            Metadata,
            str(self.get_parameter("depth_metadata_topic").value),
            qos_profile=frame_qos,
        )
        # With enable_sync + align_depth the locked driver gives all five
        # messages the same frameset header. Approximate matching at 40 ms can
        # incorrectly borrow metadata from an adjacent 30 Hz frame.
        inputs = [color, depth, color_metadata, depth_metadata, camera_info]
        names = ["color", "depth", "color_metadata", "depth_metadata", "camera_info"]
        # Register counters before the synchronizer, so the final input is
        # counted before it invokes the matched-frame callback.
        for name, subscriber in zip(names, inputs):
            subscriber.registerCallback(self._count_input, name)
        self.synchronizer = TimeSynchronizer(inputs, queue_size=frame_depth)
        self.synchronizer.registerCallback(self._on_frame)
        self.create_timer(1.0, self._publish_diagnostics)

    def _count_input(self, message, name):
        self._input_counts[name] += 1

    def _on_frame(
        self,
        color: Image,
        depth: Image,
        color_metadata: Metadata,
        depth_metadata: Metadata,
        camera_info: CameraInfo,
    ) -> None:
        self._frame_counts["matched"] += 1
        self._last_pair_monotonic = time.monotonic()
        host_receive_s = self.get_clock().now().nanoseconds * 1e-9
        try:
            color_fields = json.loads(color_metadata.json_data)
            depth_fields = json.loads(depth_metadata.json_data)
            if not isinstance(color_fields, dict) or not isinstance(depth_fields, dict):
                raise ValueError("metadata must be a JSON object")
            color_sensor_s = float(color_fields["frame_timestamp"]) * 1e-3
            depth_sensor_s = float(depth_fields["frame_timestamp"]) * 1e-3
            color_domain = str(color_fields.get("clock_domain", "unknown"))
            depth_domain = str(depth_fields.get("clock_domain", "unknown"))
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self._rejection_counts["invalid_metadata"] += 1
            self._publish_status(False, f"invalid_metadata:{exc}")
            return

        result = self.frame_clock.update(
            color_sensor_s, depth_sensor_s, color_domain, depth_domain, host_receive_s
        )
        detail = (
            f"{result.reason};color_domain={color_domain};depth_domain={depth_domain}"
            f";color_depth_delta_s={depth_sensor_s - color_sensor_s:.6f}"
        )
        self._publish_status(result.valid, detail, result.fit)
        if not result.valid:
            self._rejection_counts[result.reason] += 1
            return
        self._frame_counts["accepted"] += 1

        color_stamp = self._stamp(result.color_time_s)
        depth_stamp = self._stamp(result.depth_time_s)
        # Share immutable image buffers; only the headers are changed here.
        # Deep-copying ~1.5 MB per pair is unnecessary at 30 Hz.
        output_color = self._with_stamp(color, color_stamp)
        output_depth = self._with_stamp(depth, depth_stamp)
        self.color_publisher.publish(output_color)
        self.depth_publisher.publish(output_depth)
        output_info = self._with_stamp(camera_info, color_stamp)
        self.info_publisher.publish(output_info)

    @staticmethod
    def _with_stamp(message, stamp):
        output = copy.copy(message)
        output.header = copy.copy(message.header)
        output.header.stamp = stamp
        return output

    def _publish_diagnostics(self):
        now = time.monotonic()
        status = DiagnosticStatus()
        status.name = "d435i/exact_frame_pipeline"
        status.hardware_id = "d435i"
        stale = self._last_pair_monotonic is None or now - self._last_pair_monotonic > 0.5
        status.level = DiagnosticStatus.WARN if stale else DiagnosticStatus.OK
        status.message = "no_recent_exact_pair" if stale else "receiving_exact_pairs"
        if stale:
            # A diagnostics-only heartbeat is invisible to the mission time gate.
            # Do not leave its last valid clock evidence latched after the
            # five-message exact synchronizer stops producing pairs.
            self._publish_status(False, "no_recent_exact_pair")
        values = {"input/" + name: count for name, count in self._input_counts.items()}
        values.update({"frames/" + name: count for name, count in self._frame_counts.items()})
        values.update({"rejected/" + name: count for name, count in self._rejection_counts.items()})
        values["pair_age_s"] = "never" if self._last_pair_monotonic is None else now - self._last_pair_monotonic
        status.values = [KeyValue(key=key, value=str(value)) for key, value in sorted(values.items())]
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self.diagnostic_publisher.publish(message)

    def _publish_status(self, valid: bool, detail: str, fit: ClockFit | None = None) -> None:
        fit = self.mapper.fit if fit is None else fit
        message = TimeAlignmentStatus()
        message.header.stamp = self.get_clock().now().to_msg()
        message.sensor = "d435i_color_clock"
        message.valid = valid
        message.sample_count = fit.sample_count
        message.scale = fit.scale
        message.offset_s = fit.offset_s + self.mapper.fixed_offset_s
        message.skew_ppm = fit.skew_ppm
        message.delay_p50_s = fit.delay_p50_s
        message.delay_p95_s = fit.delay_p95_s
        message.jitter_s = fit.jitter_s
        message.reset_count = fit.reset_count
        message.detail = detail
        self.status_publisher.publish(message)

    @staticmethod
    def _stamp(seconds: float):
        from builtin_interfaces.msg import Time

        whole = math.floor(seconds)
        nanoseconds = round((seconds - whole) * 1e9)
        if nanoseconds >= 1_000_000_000:
            whole += 1
            nanoseconds -= 1_000_000_000
        return Time(sec=int(whole), nanosec=int(nanoseconds))


def main(args: list[str] | None = None) -> None:
    spin_node(D435iTimeAlignNode, args)


if __name__ == "__main__":
    main()
