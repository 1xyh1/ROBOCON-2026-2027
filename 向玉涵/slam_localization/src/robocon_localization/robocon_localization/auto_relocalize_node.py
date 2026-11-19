"""Keep a passive fixed-map localizer anchored with the existing Relocalize action."""

import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from robocon_interfaces.action import Relocalize
from robocon_interfaces.msg import LocalizationQuality, TimeAlignmentStatus
from std_msgs.msg import UInt64
from visualization_msgs.msg import Marker, MarkerArray


class AutoRelocalizeNode(Node):
    GLOBAL_TIMEOUT_S = 10.0
    LOCAL_RECOVERY_S = 0.8
    RETRY_S = 1.0

    def __init__(self):
        super().__init__("auto_relocalize")
        self._map_id = self.declare_parameter("map_id", "").value
        self._calibration_id = self.declare_parameter("calibration_id", "").value
        self._floor = self.declare_parameter("floor", "").value
        self._epoch = self.declare_parameter("odom_epoch", -1).value
        self._max_age = self.declare_parameter("maximum_scan_age_s", 0.75).value
        if not (self._map_id and self._calibration_id and self._floor and self._epoch >= 0 and self._max_age > 0):
            raise ValueError("A fixed map, calibration, floor and odometry epoch are required")

        self._clock_reset = None
        self._session_changed = False
        self._clock_seen = self._epoch_seen = self._odom_seen = float("-inf")
        self._odom_stamp = self._attempt_odom_stamp = 0
        self._quality = None
        self._last_revision = self._last_commit_stamp = 0
        self._last_commit_seen = float("-inf")
        self._last_failure = ""
        self._phase = ""

        self._future = self._goal_handle = None
        self._request_started = self._retry_at = 0.0
        self._cancel_sent = False
        self._client = ActionClient(self, Relocalize, "/localization/relocalize")
        self._marker_pub = self.create_publisher(
            MarkerArray, "/localization/recovery_markers",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.create_subscription(LocalizationQuality, "/localization/quality", self._on_quality, 10)
        self.create_subscription(TimeAlignmentStatus, "/time_alignment/status", self._on_clock, 10)
        self.create_subscription(UInt64, "/localization/odom_epoch", self._on_epoch, 10)
        self.create_subscription(Odometry, "/Odometry", self._on_odom, qos_profile_sensor_data)
        self.create_timer(0.2, self._tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    @staticmethod
    def _stamp_ns(stamp):
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    def _stamp_fresh(self, stamp_ns):
        age = self.get_clock().now().nanoseconds - stamp_ns
        return stamp_ns > 0 and -20_000_000 <= age <= self._max_age * 1_000_000_000

    def _on_clock(self, status):
        if status.sensor != "mid360_ptp_clock":
            return
        if status.valid and self._clock_reset is None:
            self._clock_reset = status.reset_count
        if self._clock_reset is not None and status.reset_count != self._clock_reset:
            self._session_changed = True
        self._clock_seen = time.monotonic() if status.valid else float("-inf")
        if not status.valid or self._session_changed:
            self._quality = None

    def _on_epoch(self, message):
        if message.data != self._epoch:
            self._session_changed = True
            self._quality = None
        self._epoch_seen = time.monotonic()

    def _on_odom(self, message):
        if message.header.frame_id != "odom" or message.child_frame_id != "mid360_imu":
            return
        stamp = self._stamp_ns(message.header.stamp)
        if stamp > self._odom_stamp:
            self._odom_stamp = stamp
            self._odom_seen = time.monotonic()

    def _on_quality(self, quality):
        if ((quality.map_id, quality.calibration_id, quality.floor, quality.odom_epoch) !=
                (self._map_id, self._calibration_id, self._floor, self._epoch)
                or quality.header.frame_id != "map"
                or quality.source not in ("relocalization", "static_map")):
            return
        stamp = self._stamp_ns(quality.header.stamp)
        if not self._stamp_fresh(stamp) or stamp < self._last_commit_stamp or self._session_changed:
            return
        if quality.accepted and quality.tf_committed:
            if quality.localization_generation > 0 and quality.tf_revision > self._last_revision:
                self._last_revision = quality.tf_revision
                self._last_commit_stamp = stamp
                self._last_commit_seen = time.monotonic()
                self._quality = quality
                self._last_failure = ""
        elif not quality.accepted:
            self._quality = None
            self._last_failure = quality.rejection_reason

    def _waiting_reason(self, wall):
        if self._session_changed:
            return "Time or odometry session changed"
        if wall - self._clock_seen > self._max_age:
            return "Waiting for MID-360 clock"
        if wall - self._epoch_seen > self._max_age:
            return "Waiting for odometry session"
        if wall - self._odom_seen > self._max_age or not self._stamp_fresh(self._odom_stamp):
            return "Waiting for fresh odometry"
        return ""

    def _start_search(self, wall):
        request = Relocalize.Goal()
        request.map_id = self._map_id
        request.calibration_id = self._calibration_id
        request.expected_floor = self._floor
        request.odom_epoch = self._epoch
        request.timeout_s = self.GLOBAL_TIMEOUT_S
        # The action interprets an empty seed array as an independent whole-map search.
        self._attempt_odom_stamp = self._odom_stamp
        self._request_started = wall
        self._cancel_sent = False
        self._last_failure = ""
        self._future = self._client.send_goal_async(request)

    def _finish_search(self, wall, reason):
        self._future = self._goal_handle = None
        self._retry_at = wall + self.RETRY_S
        if reason:
            self._last_failure = reason
            self.get_logger().info(f"Global search ended with {reason}")

    def _poll_search(self, wall):
        if self._future is None:
            return
        if not self._future.done():
            elapsed = wall - self._request_started
            if elapsed < self.GLOBAL_TIMEOUT_S + 2.0:
                return
            if (self._goal_handle is None or not self._client.server_is_ready()
                    or elapsed >= self.GLOBAL_TIMEOUT_S + 4.0):
                # The server serializes matching and enforces its own commit deadline.
                self._future.cancel()
                self._finish_search(wall, "localizer response timed out")
            elif not self._cancel_sent:
                self._goal_handle.cancel_goal_async()
                self._cancel_sent = True
            return
        try:
            response = self._future.result()
        except Exception as error:
            self._finish_search(wall, str(error))
            return
        if self._goal_handle is None:
            if response.accepted:
                self._goal_handle = response
                self._future = response.get_result_async()
            else:
                self._finish_search(wall, "request rejected")
        else:
            result = response.result
            self._finish_search(wall, "" if result.success and result.tf_committed else result.message)

    def _tick(self):
        wall = time.monotonic()
        self._poll_search(wall)
        waiting = self._waiting_reason(wall)
        if waiting:
            self._quality = None
            self._display("WAITING", waiting)
            return
        if (self._quality is not None and wall - self._last_commit_seen <= self._max_age
                and self._stamp_fresh(self._last_commit_stamp)):
            quality = self._quality
            self._display("TRACKING", f"anchor {quality.localization_generation}  "
                          f"RMSE {quality.rmse_m:.3f} m  overlap {quality.overlap_ratio:.0%}")
            return
        self._quality = None
        if (self._future is None and wall >= self._retry_at and self._odom_stamp > self._attempt_odom_stamp
                and wall - self._last_commit_seen >= self.LOCAL_RECOVERY_S
                and self._client.server_is_ready()):
            self._start_search(wall)
        if self._future is not None:
            phase = "RELOCALIZING" if self._last_revision else "GLOBAL SEARCH"
        else:
            phase = "LOST" if self._last_revision else "WAITING"
        self._display(phase, self._last_failure or "Waiting for a map match")

    def _display(self, phase, detail):
        tracking = phase == "TRACKING"
        active_search = phase in ("GLOBAL SEARCH", "RELOCALIZING")
        color = (0.22, 0.92, 0.55) if tracking else (1.0, 0.73, 0.20) if active_search else (1.0, 0.32, 0.25)
        text = Marker()
        text.header.frame_id = "mid360_imu" if tracking else "map"
        text.ns = "fixed_map_localization"
        text.id = 0
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.orientation.w = 1.0
        text.pose.position.z = 0.65 if tracking else 1.05
        text.scale.z = 0.22
        text.color.r, text.color.g, text.color.b = color
        text.color.a = 1.0
        text.text = f"{phase}\n{detail[:90]}"
        text.frame_locked = tracking
        text.lifetime.sec = 1

        arrow = Marker()
        arrow.header.frame_id = text.header.frame_id
        arrow.ns = text.ns
        arrow.id = 1
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD if tracking else Marker.DELETE
        arrow.pose.orientation.w = 1.0
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.45, 0.065, 0.065
        arrow.color.r, arrow.color.g, arrow.color.b = color
        arrow.color.a = 1.0
        arrow.frame_locked = tracking
        arrow.lifetime.sec = 1
        self._marker_pub.publish(MarkerArray(markers=[text, arrow]))

        if phase != self._phase:
            self._phase = phase
            self.get_logger().info(f"{phase} {detail}")


def main():
    rclpy.init()
    node = AutoRelocalizeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
