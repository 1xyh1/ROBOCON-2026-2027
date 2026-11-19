"""Report local odometry health; this is never global relocalization evidence."""

from __future__ import annotations

import math

import numpy as np
import rclpy
from robocon_core.ros_runtime import spin_node
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from robocon_interfaces.msg import LocalizationQuality


class LioQualityNode(Node):
    def __init__(self) -> None:
        super().__init__("lio_quality")
        self.declare_parameter("odometry_topic", "/Odometry")
        self.declare_parameter("quality_topic", "/localization/quality")
        self.declare_parameter("expected_odom_frame", "odom")
        self.declare_parameter("expected_body_frame", "mid360_imu")
        self.declare_parameter("initial_floor", "ground")
        self.declare_parameter("warmup_samples", 10)
        self.declare_parameter("max_linear_speed_mps", 5.0)
        self.declare_parameter("max_angular_speed_radps", 8.0)
        self.declare_parameter("max_position_variance_m2", 0.25)
        self.declare_parameter("max_orientation_variance_rad2", 0.25)
        self.declare_parameter("maximum_age_s", 0.25)
        self.declare_parameter("maximum_future_s", 0.02)

        self.publisher = self.create_publisher(
            LocalizationQuality, str(self.get_parameter("quality_topic").value), 10
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odometry_topic").value),
            self._on_odometry,
            20,
        )
        self.create_subscription(String, "/floor/current", self._on_floor, 10)
        self._floor = str(self.get_parameter("initial_floor").value)
        self._samples = 0
        self._last_stamp_s: float | None = None
        self._last_position: np.ndarray | None = None
        self._last_quaternion: np.ndarray | None = None

    def _on_floor(self, message: String) -> None:
        if message.data in {"ground", "l1", "l2"}:
            self._floor = message.data

    def _on_odometry(self, odometry: Odometry) -> None:
        position = np.array(
            [
                odometry.pose.pose.position.x,
                odometry.pose.pose.position.y,
                odometry.pose.pose.position.z,
            ],
            dtype=np.float64,
        )
        quaternion = np.array(
            [
                odometry.pose.pose.orientation.x,
                odometry.pose.pose.orientation.y,
                odometry.pose.pose.orientation.z,
                odometry.pose.pose.orientation.w,
            ],
            dtype=np.float64,
        )
        stamp_s = float(odometry.header.stamp.sec) + float(
            odometry.header.stamp.nanosec
        ) * 1e-9
        reasons: list[str] = []
        age_s = self.get_clock().now().nanoseconds * 1e-9 - stamp_s
        if age_s > float(self.get_parameter("maximum_age_s").value):
            reasons.append("stale_odometry")
        if age_s < -float(self.get_parameter("maximum_future_s").value):
            reasons.append("future_odometry")
        if odometry.header.frame_id != str(
            self.get_parameter("expected_odom_frame").value
        ):
            reasons.append("odom_frame")
        if odometry.child_frame_id != str(
            self.get_parameter("expected_body_frame").value
        ):
            reasons.append("body_frame")
        if not np.isfinite(position).all() or not np.isfinite(quaternion).all():
            reasons.append("non_finite_pose")

        quaternion_norm = float(np.linalg.norm(quaternion))
        if not 0.98 <= quaternion_norm <= 1.02:
            reasons.append("quaternion_norm")
        quaternion_unit = (
            quaternion / quaternion_norm if quaternion_norm > 1e-12 else quaternion
        )

        translation_step = 0.0
        rotation_step = 0.0
        if self._last_stamp_s is not None:
            dt = stamp_s - self._last_stamp_s
            if dt <= 0.0:
                reasons.append("non_monotonic_stamp")
            elif self._last_position is not None and self._last_quaternion is not None:
                translation_step = float(np.linalg.norm(position - self._last_position))
                dot = float(abs(np.dot(quaternion_unit, self._last_quaternion)))
                rotation_step = 2.0 * math.acos(float(np.clip(dot, 0.0, 1.0)))
                if translation_step / dt > float(
                    self.get_parameter("max_linear_speed_mps").value
                ):
                    reasons.append("linear_jump")
                if rotation_step / dt > float(
                    self.get_parameter("max_angular_speed_radps").value
                ):
                    reasons.append("angular_jump")

        covariance = np.asarray(odometry.pose.covariance, dtype=np.float64).reshape(6, 6)
        diagonal = np.diag(covariance)
        if not np.isfinite(covariance).all() or np.any(diagonal < 0.0):
            reasons.append("invalid_covariance")
        else:
            # An all-zero diagonal means "no uncertainty information", not
            # "perfectly known". FAST-LIVO2 used to publish this by omission.
            if float(np.max(diagonal)) <= 0.0:
                reasons.append("covariance_untrusted")
            if float(np.max(diagonal[:3])) > float(
                self.get_parameter("max_position_variance_m2").value
            ):
                reasons.append("position_variance")
            if float(np.max(diagonal[3:])) > float(
                self.get_parameter("max_orientation_variance_rad2").value
            ):
                reasons.append("orientation_variance")

        # Restart the health warmup on an integrity failure. A single quiet
        # sample following a jump must not immediately declare recovery.
        self._samples = self._samples + 1 if not reasons else 0
        converged = self._samples >= int(self.get_parameter("warmup_samples").value)
        if not converged:
            reasons.append("warming_up")

        message = LocalizationQuality()
        message.header = odometry.header
        message.accepted = not reasons
        message.converged = converged
        message.rmse_m = float("nan")
        message.overlap_ratio = float("nan")
        message.translation_step_m = translation_step
        message.rotation_step_rad = rotation_step
        message.geometry_eigenvalues = [float("nan")] * 3
        message.floor = self._floor
        message.source = (
            "fast_lio_odometry_gate" if not reasons else "fast_lio_odometry_gate:" + ",".join(reasons)
        )
        self.publisher.publish(message)

        # Only a time-valid frame may advance the comparison watermark. A
        # stale/future/non-monotonic stamp must not poison the next good frame
        # (previously one future stamp caused a chain of non_monotonic rejects).
        time_ok = not any(
            reason in {"stale_odometry", "future_odometry", "non_monotonic_stamp"}
            for reason in reasons
        )
        if time_ok:
            self._last_stamp_s = stamp_s
            self._last_position = position
            if quaternion_norm > 1e-12:
                self._last_quaternion = quaternion_unit


def main(args: list[str] | None = None) -> None:
    spin_node(LioQualityNode, args)


if __name__ == "__main__":
    main()
