"""Bounded, no-driver/no-motion fusion replay on the mini host (ROS domain 179)."""
from __future__ import annotations

import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import time

import numpy as np
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, PointCloud2
from tf2_msgs.msg import TFMessage
import yaml
from ament_index_python.packages import get_package_prefix
from robocon_core.livo_config import replay_log_failures
from robocon_core.livo_evidence import build_identity, motion_summary, timing_summary, visual_update_summary

ROOT = Path(__file__).resolve().parents[1]
ALIGNED_COLOR = "/sensors/d435i/color/image_raw"
RAW_CAMERA_TOPICS = tuple("/camera/camera/" + name for name in (
    "color/image_raw", "color/camera_info", "color/metadata",
    "aligned_depth_to_color/image_raw", "depth/metadata",
))


def stop(process, *, launch_forwards_signal=False):
    if process is None or process.poll() is not None:
        return
    # ros2 launch forwards SIGINT itself. Signalling its whole group twice can
    # interrupt Python cleanup and manufacture shutdown tracebacks.
    if launch_forwards_signal:
        process.send_signal(signal.SIGINT)
    else:
        os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=12)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture", type=Path, required=True)
    parser.add_argument("--config-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True, help="New evidence directory")
    parser.add_argument("--rate", type=float, default=0.5)
    parser.add_argument("--timeout", type=float, default=180)
    parser.add_argument("--gdb", action="store_true", help="Capture native crash backtraces")
    parser.add_argument("--with-bench-evidence", action="store_true", help="Replay recorded time-health evidence for the test console")
    args = parser.parse_args()
    if os.environ.get("ROS_DOMAIN_ID") != "179" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        parser.error("set ROS_DOMAIN_ID=179 ROS_LOCALHOST_ONLY=1")
    if not all(math.isfinite(v) and v > 0 for v in (args.rate, args.timeout)):
        parser.error("positive finite rate/timeout required")
    lock_path = ROOT / "resources/state/fusion-replay.lock"
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    replay_lock = lock_path.open("a")  # Keep alive through playback and child cleanup.
    try:
        fcntl.flock(replay_lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        parser.error("another fusion replay owns domain 179 on this host")
    bag = args.capture if (args.capture / "metadata.yaml").is_file() else args.capture / "bag"
    metadata = yaml.safe_load((bag / "metadata.yaml").read_text())
    recorded = {entry["topic_metadata"]["name"]
                for entry in metadata["rosbag2_bagfile_information"]["topics_with_message_count"]
                if entry["message_count"]}
    raw_camera = ALIGNED_COLOR not in recorded
    inputs = ("/livox/lidar", "/livox/imu", *(RAW_CAMERA_TOPICS if raw_camera else (ALIGNED_COLOR,)))
    missing = sorted(set(inputs) - recorded)
    if missing:
        parser.error("missing recorded sensor inputs: " + ", ".join(missing))
    if args.timeout < metadata["rosbag2_bagfile_information"]["duration"]["nanoseconds"] / 1e9 / args.rate + 15:
        parser.error("timeout must cover the complete bag and startup/drain allowance")
    args.output.mkdir(parents=True, exist_ok=False)
    executable = Path(get_package_prefix("fast_livo")) / "lib/fast_livo/fastlivo_mapping"
    source_marker = ROOT / "src/experimental/fast_livo/.robocon-source.json"
    identity = build_identity(executable, {"FAST-LIVO2-ROS2": json.loads(source_marker.read_text())})
    rclpy.init()
    node = rclpy.create_node("livo_replay_probe", parameter_overrides=[Parameter("use_sim_time", value=True)])
    discovery_deadline = time.monotonic() + 1.5
    while time.monotonic() < discovery_deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    occupied = [topic for topic in ("/clock", "/livox/lidar", "/livox/imu",
                                  "/fusion/input/color_rect", "/fusion/livo/aft_mapped_to_init")
                if node.get_publishers_info_by_topic(topic)]
    if occupied:
        node.destroy_node()
        rclpy.shutdown()
        parser.error("replay domain already has publishers: " + ", ".join(occupied))
    odometry, ages, cloud_stamps = [], [], []
    counts = {"rectified_images": 0, "visualized_images": 0, "tf_messages": 0}
    invalid = []

    def odom(message):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        if (not all(math.isfinite(v) for v in values)
                or message.header.frame_id != "livo_odom" or message.child_frame_id != "livo_mid360_imu"):
            invalid.append("invalid_pose_or_frame")
        odometry.append((stamp, *values))
        ages.append(node.get_clock().now().nanoseconds / 1e9 - stamp)

    def count(name):
        def receive(_message):
            counts[name] += 1
        return receive

    subscriptions = [
        node.create_subscription(Odometry, "/fusion/livo/aft_mapped_to_init", odom, qos_profile_sensor_data),
        node.create_subscription(Image, "/fusion/input/color_rect", count("rectified_images"), qos_profile_sensor_data),
        node.create_subscription(Image, "/fusion/livo/rgb_img", count("visualized_images"), qos_profile_sensor_data),
        node.create_subscription(PointCloud2, "/fusion/livo/cloud_registered",
                                 lambda m: cloud_stamps.append(m.header.stamp.sec + m.header.stamp.nanosec / 1e9),
                                 qos_profile_sensor_data),
        *[node.create_subscription(TFMessage, topic, count("tf_messages"), qos_profile_sensor_data)
          for topic in ("/tf", "/tf_static", "/fusion/livo/tf", "/fusion/livo/tf_static")],
    ]
    launch_process = bag_process = None
    error = ""
    playback_completed = False
    start = time.monotonic()
    with (args.output / "launch.log").open("w") as launch_log, (args.output / "playback.log").open("w") as bag_log:
        try:
            launch_process = subprocess.Popen([
                "ros2", "launch", "robocon_fusion", "livo_replay.launch.py",
                f"config_dir:={args.config_dir.resolve()}", f"gdb:={str(args.gdb).lower()}",
                f"raw_camera:={str(raw_camera).lower()}"], stdout=launch_log, stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL, start_new_session=True)
            deadline = time.monotonic() + 15
            while time.monotonic() < deadline:
                if launch_process.poll() is not None:
                    raise RuntimeError("fusion launch exited during discovery")
                if all(node.get_subscriptions_info_by_topic(topic) for topic in
                       (*inputs, ALIGNED_COLOR, "/fusion/input/color_rect")):
                    break
                rclpy.spin_once(node, timeout_sec=0.1)
            else:
                raise RuntimeError("fusion input subscribers not ready")
            bag_process = subprocess.Popen([
                "ros2", "bag", "play", str(bag.resolve()),
                "--clock", "100", "--rate", str(args.rate), "--topics",
                *inputs, *(["/livox/clock_device"] if args.with_bench_evidence else []),
                *(["/time_alignment/status"] if args.with_bench_evidence and not raw_camera else [])],
                stdout=bag_log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, start_new_session=True)
            while time.monotonic() - start < args.timeout:
                rclpy.spin_once(node, timeout_sec=0.02)
                if launch_process.poll() is not None:
                    raise RuntimeError("fusion launch exited during playback")
                if bag_process.poll() is not None:
                    playback_completed = bag_process.returncode == 0
                    if not playback_completed:
                        raise RuntimeError(f"bag playback failed: {bag_process.returncode}")
                    drain_deadline = time.monotonic() + 3
                    while time.monotonic() < drain_deadline:
                        rclpy.spin_once(node, timeout_sec=0.05)
                    break
            else:
                raise RuntimeError("replay timeout")
        except (RuntimeError, KeyboardInterrupt) as exc:
            error = str(exc) or "interrupted"
        finally:
            stop(bag_process)
            stop(launch_process, launch_forwards_signal=True)
            node.destroy_node()
            rclpy.shutdown()
    log = (args.output / "launch.log").read_text(errors="replace")
    exits = replay_log_failures(log)
    shutdown_clean = (not exits["child_exit_errors"] and not exits["debugger_stop_signals"]
                      and launch_process is not None and launch_process.returncode == 0)
    retrievals = [int(value) for value in re.findall(r"Retrieve (\d+) points from visual sparse map", log)]
    stamps = np.asarray([row[0] for row in odometry])
    deltas = np.diff(stamps)
    span = float(stamps[-1] - stamps[0]) if len(stamps) > 1 else 0.0
    report = {
        "kind": "isolated_real_sensor_bag_livo_replay_not_accuracy_acceptance",
        "capture": str(args.capture.resolve()), "config_dir": str(args.config_dir.resolve()),
        "camera_input": "raw_exact_frameset" if raw_camera else "aligned_color", "sensor_inputs": list(inputs),
        "build_identity": identity,
        "pass_scope": "transport_timing_and_process_exit_only",
        "estimation_accuracy_evaluated": False,
        "calibration_evidence": json.loads((args.config_dir / "evidence.json").read_text()),
        "playback_rate": args.rate, "elapsed_wall_s": time.monotonic() - start,
        "bag_receipt_span_s": metadata["rosbag2_bagfile_information"]["duration"]["nanoseconds"] / 1e9,
        "playback_completed": playback_completed, "error": error, "invalid_output": invalid,
        **exits, "shutdown_clean": shutdown_clean,
        **counts, "odometry_count": len(odometry), "cloud_count": len(cloud_stamps),
        "odometry_measurement_span_s": span,
        "odometry_measurement_hz": (len(stamps) - 1) / span if span > 0 else 0,
        "max_odometry_gap_s": float(deltas.max()) if len(deltas) else None,
        "non_increasing_stamps": int((deltas <= 0).sum()),
        "sim_clock_age_p50_s": float(np.percentile(ages, 50)) if ages else None,
        "sim_clock_age_p99_s": float(np.percentile(ages, 99)) if ages else None,
        "sim_clock_age_min_s": float(min(ages)) if ages else None,
        "positive_visual_constraint_frames": sum(n > 0 for n in retrievals),
        "visual_constraint_points_max": max(retrievals, default=0),
        "net_translation_m_not_accuracy": float(np.linalg.norm(np.asarray(odometry[-1][1:4])
                                                              - odometry[0][1:4])) if odometry else None,
        "launch_exit_code_after_stop": launch_process.returncode if launch_process else None,
        "estimated_motion": motion_summary(odometry),
        "timing_diagnostics": timing_summary(log),
        "visual_update_diagnostics": visual_update_summary(log),
    }
    report["freshness_gate_passed"] = bool(
        len(deltas) and deltas.max() <= 0.75 and ages
        and min(ages) >= -0.02 and np.percentile(ages, 99) <= 0.75)
    report["replay_chain_passed"] = bool(
        playback_completed and shutdown_clean and not error and not invalid and len(odometry) >= 30 and span > 10
        and report["freshness_gate_passed"]
        and counts["rectified_images"] > 30 and counts["visualized_images"] > 30
        and report["positive_visual_constraint_frames"] > 0 and counts["tf_messages"] == 0
        and report["non_increasing_stamps"] == 0)
    (args.output / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    (args.output / "odometry.json").write_text(json.dumps(odometry, allow_nan=False) + "\n")
    print(json.dumps(report, indent=2, allow_nan=False))
    return 0 if report["replay_chain_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
