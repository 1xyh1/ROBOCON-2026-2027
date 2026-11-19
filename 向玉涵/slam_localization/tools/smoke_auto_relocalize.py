"""Humble smoke test of the actual GICP action and automatic recovery with synthetic sensors."""
from contextlib import ExitStack
from math import atan2, cos, dist, pi, sin
import os
from pathlib import Path
from random import Random
import signal
import subprocess
import tempfile
import time

os.environ["ROS_DOMAIN_ID"] = "178"
os.environ["ROS_LOCALHOST_ONLY"] = "1"

import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from robocon_interfaces.msg import LocalizationQuality, TimeAlignmentStatus
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header, UInt64
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray


def scene():
    random = Random(20261112)
    reference, views = [], []
    for index, (cx, cy, cz, degrees) in enumerate(((0, 0, 0, 72), (9, 2, 0.4, -128), (-7, 6, 0.1, 153))):
        points = [(cx + random.uniform(-2.1, 2.1), cy + random.uniform(-1.5, 1.5),
                   cz + 0.3 + random.uniform(-1.0, 1.0)) for _ in range(480)]
        reference.extend(points)
        truth = cx + 0.25 + 0.13 * index, cy - 0.27, cz + 0.1
        heading = degrees * pi / 180
        c, s = cos(heading), sin(heading)
        view = []
        for px, py, pz in points[:340]:
            x, y, z = px - truth[0], py - truth[1], pz - truth[2]
            view.append((c*x + s*y, -s*x + c*y, z))
        views.append((truth, heading, view))
    return reference, views


def write_pcd(path, points):
    path.write_text(
        f"VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        f"WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA ascii\n"
        + "".join(f"{x:.8f} {y:.8f} {z:.8f}\n" for x, y, z in points), encoding="ascii")


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=15)


def main(*, force_loss_between_anchors=False):
    reference, views = scene()
    with ExitStack() as stack:
        folder = Path(stack.enter_context(tempfile.TemporaryDirectory(prefix="auto-reloc-smoke-")))
        pcd = folder/"reference.pcd"
        write_pcd(pcd, reference)
        parameters = ["--ros-args", "-p", "map_id:=synthetic-auto-map", "-p", "calibration_id:=synthetic-auto-lio",
                      "-p", "floor:=ground", "-p", "odom_epoch:=42"]
        for package, executable, extra in (
            ("robocon_global_localization", "global_localizer_node",
             ["-p", f"map_pcd:={pcd}", "-p", "publish_tf:=true", "-p", "continuous_enabled:=true"]),
            ("robocon_localization", "auto_relocalize_node", []),
        ):
            prefix = Path(subprocess.check_output(["ros2", "pkg", "prefix", package], text=True).strip())
            path = prefix/"lib"/package/executable
            process = subprocess.Popen([str(path), *parameters, *extra], start_new_session=True)
            stack.callback(stop, process)
        rclpy.init()
        stack.callback(rclpy.try_shutdown)
        node = rclpy.create_node("recovery_smoke_input")
        stack.callback(node.destroy_node)

        odom_pub = node.create_publisher(Odometry, "/Odometry", qos_profile_sensor_data)
        cloud_pub = node.create_publisher(PointCloud2, "/cloud_registered_body", qos_profile_sensor_data)
        imu_pub = node.create_publisher(Imu, "/livox/imu", qos_profile_sensor_data)
        clock_pub = node.create_publisher(TimeAlignmentStatus, "/time_alignment/status", 10)
        epoch_pub = node.create_publisher(UInt64, "/localization/odom_epoch", 10)
        transforms, qualities, markers = {}, [], []
        def key(stamp):
            return stamp.sec, stamp.nanosec
        def on_tf(message):
            for transform in message.transforms:
                if transform.header.frame_id == "map" and transform.child_frame_id == "odom":
                    transforms[key(transform.header.stamp)] = transform.transform
        node.create_subscription(TFMessage, "/tf", on_tf, 10)
        node.create_subscription(LocalizationQuality, "/localization/quality", qualities.append, 10)
        node.create_subscription(MarkerArray, "/localization/recovery_markers", markers.append, 10)

        def drive(cloud, seconds, until=lambda: False):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                stamp = node.get_clock().now().to_msg()
                epoch_pub.publish(UInt64(data=42))
                clock_pub.publish(TimeAlignmentStatus(sensor="mid360_ptp_clock", valid=True, reset_count=0))
                imu = Imu(header=Header(stamp=stamp, frame_id="mid360_imu"))
                imu.linear_acceleration.z = 9.81
                imu_pub.publish(imu)
                # This odometry stays at the origin while the observed viewpoint is moved.
                odom = Odometry(header=Header(stamp=stamp, frame_id="odom"), child_frame_id="mid360_imu")
                odom.pose.pose.orientation.w = 1.0
                odom_pub.publish(odom)
                cloud_pub.publish(create_cloud_xyz32(Header(stamp=stamp, frame_id="mid360_imu"), cloud))
                for _ in range(3):
                    rclpy.spin_once(node, timeout_sec=0.022)
                if until():
                    return True
                time.sleep(0.04)
            return False

        for index, (truth, heading, view) in enumerate(views, start=1):
            if index == 3 or (force_loss_between_anchors and index > 1):
                # Too little geometry must leave the pose arrow hidden.
                marker_start = len(markers)
                drive([(0.0, 0.0, 0.0)], 1.7)
                assert any(any(item.id == 1 and item.action == Marker.DELETE for item in event.markers)
                           for event in markers[marker_start:]), "lost pose arrow stayed visible"
            start = len(qualities)
            def committed():
                return next((q for q in qualities[start:] if q.source == "relocalization" and q.accepted and
                             q.tf_committed and q.localization_generation == index and
                             key(q.header.stamp) in transforms), None)
            assert drive(view, 24.0, until=committed), f"anchor {index} did not recover automatically"
            quality = committed()
            estimate = transforms[key(quality.header.stamp)]
            position_error = dist(truth, (estimate.translation.x, estimate.translation.y, estimate.translation.z))
            q = estimate.rotation
            yaw = atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            heading_error = abs(atan2(sin(yaw-heading), cos(yaw-heading))) * 180/pi
            assert position_error < 0.15 and heading_error < 4.0, (position_error, heading_error)
            drive(view, 1.3)
            assert any(q.source == "static_map" and q.accepted and q.tf_committed
                       and q.localization_generation == index for q in qualities[start:])
            assert any(any(item.text.startswith(f"TRACKING\nanchor {index} ") for item in event.markers)
                       for event in markers)
            print(f"anchor {index} position_error_m={position_error:.4f} heading_error_deg={heading_error:.3f}")


if __name__ == "__main__":
    main()
