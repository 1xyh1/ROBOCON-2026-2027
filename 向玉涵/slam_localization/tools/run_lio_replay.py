"""Replay MID-360 point clouds and its IMU through the installed FAST-LIO.

Uses the same capture format, process lock, domain and trajectory representation
as run_livo_replay.py. No camera, device driver or motion command is started.
"""
from __future__ import annotations

import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import subprocess
import time

import numpy as np
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2
from tf2_msgs.msg import TFMessage
from livox_ros_driver2.msg import CustomMsg
from ament_index_python.packages import get_package_prefix, get_package_share_directory
import yaml

from robocon_core.livo_evidence import motion_summary, sha256
from run_livo_replay import stop

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True, help="New evidence directory")
    parser.add_argument("--rate", type=float, default=1.0)
    parser.add_argument("--timeout", type=float, default=330.0)
    args = parser.parse_args()
    if os.environ.get("ROS_DOMAIN_ID") != "179" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        parser.error("set ROS_DOMAIN_ID=179 ROS_LOCALHOST_ONLY=1")
    if not all(math.isfinite(v) and v > 0 for v in (args.rate, args.timeout)):
        parser.error("positive finite rate/timeout required")

    bag = args.capture if (args.capture / "metadata.yaml").is_file() else args.capture / "bag"
    metadata_path = bag / "metadata.yaml"
    metadata = yaml.safe_load(metadata_path.read_text())["rosbag2_bagfile_information"]
    duration = metadata["duration"]["nanoseconds"] / 1e9
    if args.timeout < duration / args.rate + 15:
        parser.error("timeout must cover the complete bag and startup/drain allowance")
    topics = {item["topic_metadata"]["name"]: item for item in metadata["topics_with_message_count"]}
    for topic, expected_type in (("/livox/lidar", "livox_ros_driver2/msg/CustomMsg"),
                                 ("/livox/imu", "sensor_msgs/msg/Imu")):
        if topics[topic]["topic_metadata"]["type"] != expected_type:
            parser.error(f"unexpected recorded type for {topic}")

    lock_path = ROOT / "resources/state/fusion-replay.lock"
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    replay_lock = lock_path.open("a")
    try:
        fcntl.flock(replay_lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        parser.error("another comparison replay owns domain 179 on this host")
    args.output.mkdir(parents=True, exist_ok=False)

    executable = Path(get_package_prefix("fast_lio")) / "lib/fast_lio/fastlio_mapping"
    config_source = Path(get_package_share_directory("robocon_bringup")) / "config/fast_lio_mid360.yaml"
    config = yaml.safe_load(config_source.read_text())
    params = config["/**"]["ros__parameters"]
    # Preserve the installed estimation parameters; the comparison has no TF authority.
    params["use_sim_time"] = True
    params["common"]["publish_tf"] = False
    params["pcd_save"]["pcd_save_en"] = False
    config_path = args.output / "lio.yaml"
    config_path.write_text(yaml.safe_dump(config, sort_keys=False))

    rclpy.init()
    node = rclpy.create_node("lio_replay_probe", parameter_overrides=[Parameter("use_sim_time", value=True)])
    discovery_deadline = time.monotonic() + 1.5
    while time.monotonic() < discovery_deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    occupied = [topic for topic in ("/clock", "/livox/lidar", "/livox/imu", "/comparison/lio/odometry")
                if node.get_publishers_info_by_topic(topic)]
    if occupied:
        node.destroy_node()
        rclpy.shutdown()
        parser.error("replay domain already has publishers: " + ", ".join(occupied))

    counts = {"received_lidar": 0, "received_imu": 0, "cloud_messages": 0, "tf_messages": 0}
    odometry, ages, covariance_diagonals = [], [], []
    invalid = []

    def count(name):
        def receive(_message):
            counts[name] += 1
        return receive

    def on_odom(message):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        if (not all(math.isfinite(v) for v in values)
                or message.header.frame_id != params["common"]["odom_frame"]
                or message.child_frame_id != params["common"]["body_frame"]):
            invalid.append("invalid_pose_or_frame")
        covariance = np.asarray(message.pose.covariance).reshape(6, 6)
        if not np.isfinite(covariance).all():
            invalid.append("nonfinite_pose_covariance")
        else:
            covariance_diagonals.append(covariance.diagonal().tolist())
        odometry.append((stamp, *values))
        ages.append(node.get_clock().now().nanoseconds / 1e9 - stamp)

    subscriptions = [
        node.create_subscription(CustomMsg, "/livox/lidar", count("received_lidar"), qos_profile_sensor_data),
        node.create_subscription(Imu, "/livox/imu", count("received_imu"), qos_profile_sensor_data),
        node.create_subscription(Odometry, "/comparison/lio/odometry", on_odom, qos_profile_sensor_data),
        node.create_subscription(PointCloud2, "/comparison/lio/cloud_registered", count("cloud_messages"), qos_profile_sensor_data),
        *[node.create_subscription(TFMessage, topic, count("tf_messages"), qos_profile_sensor_data)
          for topic in ("/tf", "/tf_static", "/comparison/lio/tf", "/comparison/lio/tf_static")],
    ]

    frontend = playback = None
    error = ""
    playback_completed = False
    start = time.monotonic()
    environment = dict(os.environ, ROS_LOG_DIR=str(args.output.resolve() / "ros-logs"))
    with (args.output / "frontend.log").open("w") as frontend_log, (args.output / "playback.log").open("w") as playback_log:
        try:
            frontend = subprocess.Popen([
                str(executable), "--ros-args", "--params-file", str(config_path.resolve()),
                "-r", "/Odometry:=/comparison/lio/odometry",
                *[item for topic in ("/cloud_registered", "/cloud_registered_body", "/cloud_effected", "/Laser_map", "/path", "/tf", "/tf_static")
                  for item in ("-r", topic + ":=/comparison/lio" + topic)],
            ], cwd=args.output, env=environment, stdout=frontend_log, stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL, start_new_session=True)
            deadline = time.monotonic() + 15
            while time.monotonic() < deadline:
                if frontend.poll() is not None:
                    raise RuntimeError("FAST-LIO exited during discovery")
                # The probe has its own subscriptions; count the algorithm separately.
                if all(len(node.get_subscriptions_info_by_topic(topic)) >= 2
                       for topic in ("/livox/lidar", "/livox/imu")):
                    break
                rclpy.spin_once(node, timeout_sec=0.1)
            else:
                raise RuntimeError("FAST-LIO input subscribers not ready")
            playback = subprocess.Popen([
                "ros2", "bag", "play", str(bag.resolve()),
                "--clock", "100", "--rate", str(args.rate), "--topics", "/livox/lidar", "/livox/imu",
            ], env=environment, stdout=playback_log, stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL, start_new_session=True)
            while time.monotonic() - start < args.timeout:
                rclpy.spin_once(node, timeout_sec=0.02)
                if frontend.poll() is not None:
                    raise RuntimeError("FAST-LIO exited during playback")
                if playback.poll() is not None:
                    playback_completed = playback.returncode == 0
                    if not playback_completed:
                        raise RuntimeError(f"bag playback failed: {playback.returncode}")
                    deadline = time.monotonic() + 3
                    while time.monotonic() < deadline:
                        rclpy.spin_once(node, timeout_sec=0.05)
                    break
            else:
                raise RuntimeError("replay timeout")
        except (RuntimeError, KeyboardInterrupt) as exc:
            error = str(exc) or "interrupted"
        finally:
            stop(playback)
            stop(frontend)
            node.destroy_node()
            rclpy.shutdown()

    stamps = np.asarray([row[0] for row in odometry])
    deltas = np.diff(stamps)
    span = float(stamps[-1] - stamps[0]) if len(stamps) > 1 else 0.0
    report = {
        "kind": "isolated_mid360_lidar_imu_relative_odometry_replay",
        "pass_scope": "transport_timing_finite_output_and_process_exit_only",
        "estimation_accuracy_evaluated": False,
        "capture": str(args.capture.resolve()), "bag": str(bag.resolve()),
        "bag_metadata_sha256": sha256(metadata_path), "bag_receipt_span_s": duration,
        "executable": str(executable), "executable_sha256": sha256(executable),
        "source_identity_verified": False,
        "source_identity_reason": "retained_installed_binary_without_build_bound_manifest",
        "config_source": str(config_source), "config_source_sha256": sha256(config_source),
        "effective_config_sha256": sha256(config_path), "runner_sha256": sha256(Path(__file__)),
        "played_topics": ["/livox/lidar", "/livox/imu"], "camera_used": False,
        "playback_rate": args.rate, "elapsed_wall_s": time.monotonic() - start,
        "playback_completed": playback_completed, "error": error, "invalid_output": invalid,
        "frontend_exit_code": frontend.returncode if frontend else None,
        "expected_lidar_messages": topics["/livox/lidar"]["message_count"],
        "expected_imu_messages": topics["/livox/imu"]["message_count"],
        **counts,
        "odometry_count": len(odometry), "odometry_measurement_span_s": span,
        "odometry_measurement_hz": (len(stamps) - 1) / span if span > 0 else 0,
        "max_odometry_gap_s": float(deltas.max()) if len(deltas) else None,
        "non_increasing_stamps": int((deltas <= 0).sum()),
        "sim_clock_age_p50_s": float(np.percentile(ages, 50)) if ages else None,
        "sim_clock_age_p99_s": float(np.percentile(ages, 99)) if ages else None,
        "sim_clock_age_min_s": float(min(ages)) if ages else None,
        "pose_covariance_diagonal_max": np.max(covariance_diagonals, axis=0).tolist() if covariance_diagonals else None,
        "estimated_motion": motion_summary(odometry),
    }
    report["freshness_gate_passed"] = bool(len(deltas) and deltas.max() <= 0.75 and ages
        and min(ages) >= -0.02 and np.percentile(ages, 99) <= 0.75)
    report["replay_chain_passed"] = bool(playback_completed and not error and not invalid
        and frontend is not None and frontend.returncode == 0 and len(odometry) >= 30 and span > 10
        and report["freshness_gate_passed"] and report["non_increasing_stamps"] == 0 and counts["tf_messages"] == 0)
    (args.output / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    (args.output / "odometry.json").write_text(json.dumps(odometry, allow_nan=False) + "\n")
    print(json.dumps(report, indent=2, allow_nan=False))
    return 0 if report["replay_chain_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
