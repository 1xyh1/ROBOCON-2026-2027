"""Headless acquisition/assessment backend for the desktop SLAM bench."""
from __future__ import annotations

import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import shutil
import subprocess
import sys
import time

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, Imu, PointCloud2
from livox_ros_driver2.msg import CustomMsg
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
from robocon_interfaces.msg import TimeAlignmentStatus
from ament_index_python.packages import get_package_prefix
import yaml

from robocon_core.livo_config import replay_log_failures
from robocon_core.livo_evidence import VisualUpdateAccumulator, visual_update_summary
from robocon_core.slam_bench import BenchMetrics, evaluate, xyz_from_cloud, stamp_from_cdr


def save(path, document):
    if path.name == "state.json":
        document = dict(document, heartbeat_utc_ns=time.time_ns(), backend_pid=os.getpid())
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(document, ensure_ascii=False, allow_nan=False), encoding="utf-8")
    temporary.replace(path)


def stop(process):
    if process is None or process.poll() is not None:
        return False
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=20)
        return False
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)
        return True


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", required=True, type=Path)
    parser.add_argument("--config-dir", required=True, type=Path)
    parser.add_argument("--standard", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--source", choices=("live", "replay"), default="live")
    parser.add_argument("--capture", type=Path)
    parser.add_argument("--case", default="SB01")
    parser.add_argument("--duration", type=float, default=180)
    parser.add_argument("--warmup", type=float, default=10)
    parser.add_argument("--reference-lio", action="store_true")
    parser.add_argument("--stationary-confirmed", action="store_true", help="Operator confirms the fixed, untouched SB01 condition")
    options = parser.parse_args(args)
    standard = yaml.safe_load(options.standard.read_text())
    if options.case not in standard["cases"] or not 5 <= options.duration <= 1800 or not 0 <= options.warmup <= 60:
        parser.error("invalid case/duration/warmup")
    if options.source == "replay" and options.capture is None:
        parser.error("replay needs a capture directory")
    repo, output = options.repo.resolve(), options.output.resolve()
    domain = "178" if options.source == "live" else "179"
    os.environ.update(ROS_DOMAIN_ID=domain, ROS_LOCALHOST_ONLY="1")
    lock_path = repo / f"robot_ws/state/slam-bench-{domain}.lock"
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    owner_lock = lock_path.open("a")
    try:
        fcntl.flock(owner_lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        parser.error("another test session already owns this domain")
    output.mkdir(parents=True, exist_ok=False)
    calibration = json.loads((options.config_dir / "evidence.json").read_text())
    identity = {"source": options.source, "case_id": options.case, "domain": domain,
                "requested_duration_s": options.duration, "warmup_s": options.warmup,
                "standard": standard, "standard_sha256": hashlib.sha256(options.standard.read_bytes()).hexdigest(),
                "calibration": calibration, "config_dir": str(options.config_dir.resolve()),
                "started_utc_ns": time.time_ns(), "robot_motion_commanded": False,
                "reference_lio": options.reference_lio}
    executable = Path(get_package_prefix("fast_livo")) / "lib/fast_livo/fastlivo_mapping"
    identity["executable_sha256"] = hashlib.sha256(executable.read_bytes()).hexdigest()
    identity["backend_sha256"] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    identity["device_preflight_verified"] = options.source == "live"
    identity["stationarity_is_protocol_assumption_not_independent_truth"] = options.case == "SB01"
    identity["operator_stationarity_confirmed"] = options.stationary_confirmed
    save(output / "manifest.json", identity)
    save(output / "state.json", {"phase": "preflight", "identity": identity})
    stopping = [False]
    signal.signal(signal.SIGTERM, lambda *_: stopping.__setitem__(0, True))
    signal.signal(signal.SIGINT, lambda *_: stopping.__setitem__(0, True))
    original_parent = os.getppid()
    if options.source == "live":
        processes = subprocess.check_output(["ps", "-eo", "pid=,args="], text=True)
        busy = [line for line in processes.splitlines() if len(line.split()) > 1 and
                Path(line.split()[1]).name in ("livox_ros_driver2_node", "realsense2_camera_node", "realsense-viewer")]
        if busy:
            save(output / "state.json", {"phase": "failed", "error": "设备已被其他进程占用；未改配置/抢占设备", "identity": identity})
            (output / "busy_processes.log").write_text("\n".join(busy))
            return 1
        camera = json.loads((options.config_dir / "camera.json").read_text())
        preflight = subprocess.run([sys.executable, str(repo / "scripts/preflight_hardware.py"),
                                   "--camera-serial", camera["camera_serial"], "--output", str(output / "preflight.json")],
                                  capture_output=True, text=True, timeout=35)
        (output / "preflight.log").write_text(preflight.stdout + preflight.stderr)
        if preflight.returncode:
            save(output / "state.json", {"phase": "failed", "error": "硬件预检失败，未启动测试", "identity": identity})
            return 1
    rclpy.init(args=[])
    node = rclpy.create_node("slam_bench_monitor", parameter_overrides=[Parameter("use_sim_time", value=options.source == "replay")])
    discovery = time.monotonic() + 1.5
    while time.monotonic() < discovery:
        rclpy.spin_once(node, timeout_sec=.1)
    occupied = [t for t in ("/livox/lidar", "/fusion/livo/aft_mapped_to_init", "/clock")
                if node.get_publishers_info_by_topic(t)]
    if occupied:
        save(output / "state.json", {"phase": "failed", "error": "测试域已被占用: " + str(occupied), "identity": identity})
        node.destroy_node()
        rclpy.shutdown()
        return 1
    metrics = BenchMetrics()
    measuring_from = None
    last_seen, cloud, device_clock = {}, [], {}
    cloud_map = {}
    preview_at = {"image": 0., "cloud": 0.}
    bridge = CvBridge()
    tf_count = [0]
    visual_frames, visual_points_max, log_position, partial_line = 0, 0, 0, ""
    visual_updates = VisualUpdateAccumulator()
    samples = (output / "samples.jsonl").open("w", encoding="utf-8")

    def receive_stamp(name, stamp, pose=None):
        now = time.monotonic()
        last_seen[name] = now
        if measuring_from is None:
            return
        elapsed = now - measuring_from
        age = node.get_clock().now().nanoseconds / 1e9 - stamp
        metrics.add(name, stamp, elapsed, age=age if name == "odom" else None, pose=pose)
        samples.write(json.dumps({"stream": name, "stamp": stamp, "elapsed": elapsed,
                                  "age": age if name == "odom" else None, "pose": pose}) + "\n")

    def receive(name, message, pose=None):
        receive_stamp(name, message.header.stamp.sec + message.header.stamp.nanosec / 1e9, pose)

    def odometry(message, name="odom"):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        if name == "odom" and (message.header.frame_id != "livo_odom" or message.child_frame_id != "livo_mid360_imu"):
            metrics.invalid += 1
            return
        receive(name, message, [p.x, p.y, p.z, q.x, q.y, q.z, q.w] if name == "odom" else None)

    def image(serialized):
        if time.monotonic() - preview_at["image"] < .25:
            return
        preview_at["image"] = time.monotonic()
        message = deserialize_message(serialized, Image)
        frame = bridge.imgmsg_to_cv2(message, "bgr8")
        ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
        if ok:
            temporary = output / "image.jpg.tmp"
            temporary.write_bytes(encoded.tobytes())
            temporary.replace(output / "image.jpg")

    def points(message):
        nonlocal cloud
        if time.monotonic() - preview_at["cloud"] < .3:
            return
        preview_at["cloud"] = time.monotonic()
        if message.header.frame_id != "livo_odom":
            metrics.invalid += 1
            return
        xyz = xyz_from_cloud(message.data, message.fields, message.point_step, message.row_step,
                             message.width, message.height, message.is_bigendian)
        for point in xyz:
            if np.linalg.norm(point) < 40:
                cloud_map[tuple(np.floor(point / .1).astype(int))] = np.round(point, 3).tolist()
        while len(cloud_map) > 25000:
            cloud_map.pop(next(iter(cloud_map)))
        values = list(cloud_map.values())
        cloud = values[::max(1, len(values) // 6000)]

    def clock(message):
        nonlocal device_clock
        device_clock = json.loads(message.data)

    def health(message):
        if measuring_from is not None:
            metrics.add_health(message.sensor, time.monotonic() - measuring_from, message.valid)

    subscriptions = [
        node.create_subscription(Odometry, "/fusion/livo/aft_mapped_to_init", odometry, qos_profile_sensor_data),
        node.create_subscription(Odometry, "/bench/reference/odom", lambda m: odometry(m, "reference"), qos_profile_sensor_data),
        node.create_subscription(CustomMsg, "/livox/lidar", lambda m: receive_stamp("lidar", stamp_from_cdr(m)),
                                 QoSProfile(depth=16, reliability=ReliabilityPolicy.BEST_EFFORT), raw=True),
        node.create_subscription(Imu, "/livox/imu", lambda m: receive("imu", m),
                                 QoSProfile(depth=256, reliability=ReliabilityPolicy.BEST_EFFORT)),
        node.create_subscription(Image, "/sensors/d435i/color/image_raw", lambda m: receive_stamp("rgb", stamp_from_cdr(m)),
                                 QoSProfile(depth=8, reliability=ReliabilityPolicy.RELIABLE), raw=True),
        node.create_subscription(Image, "/fusion/livo/rgb_img", image, qos_profile_sensor_data, raw=True),
        node.create_subscription(PointCloud2, "/fusion/livo/cloud_registered", points, qos_profile_sensor_data),
        node.create_subscription(String, "/livox/clock_device", clock, 10),
        node.create_subscription(TimeAlignmentStatus, "/time_alignment/status", health, 20),
        *[node.create_subscription(TFMessage, topic, lambda m: tf_count.__setitem__(0, tf_count[0] + 1), qos_profile_sensor_data)
          for topic in ("/tf", "/tf_static", "/fusion/livo/tf", "/fusion/livo/tf_static")],
    ]
    process, error, completed, forced = None, "", False, False
    if options.source == "live":
        command = ["ros2", "launch", "robocon_fusion", "bench_live.launch.py", f"repo:={repo}",
                   f"config_dir:={options.config_dir.resolve()}", f"reference_lio:={str(options.reference_lio).lower()}"]
    else:
        command = [sys.executable, str(repo / "scripts/run_livo_replay.py"), "--capture", str(options.capture.resolve()),
                   "--config-dir", str(options.config_dir.resolve()), "--output", str(output / "replay"),
                   "--rate", "1", "--timeout", str(max(options.duration + 90, 180)), "--with-bench-evidence"]
    start, ready_at, last_snapshot, measured_elapsed = time.monotonic(), None, 0., 0.
    with (output / "launch.log").open("w", encoding="utf-8") as launch_log:
        try:
            process = subprocess.Popen(command, cwd=repo, stdout=launch_log, stderr=subprocess.STDOUT,
                                       stdin=subprocess.DEVNULL, start_new_session=True)
            while not stopping[0]:
                rclpy.spin_once(node, timeout_sec=.01)
                now = time.monotonic()
                if os.getppid() != original_parent or (output / "stop.request").exists():
                    stopping[0] = True
                    break
                if now - start > 40 and ready_at is None:
                    raise RuntimeError("40 秒内数据链未就绪；未开始正式采样")
                if ready_at is None and all(now - last_seen.get(key, -100) < .75 for key in ("odom", "lidar", "imu", "rgb")):
                    ready_at = now
                warmup = options.warmup if options.source == "live" else 0.
                if ready_at is not None and measuring_from is None and now - ready_at >= warmup:
                    measuring_from = now
                    metrics = BenchMetrics()
                measured_elapsed = max(0., now - measuring_from) if measuring_from is not None else 0.
                if options.source == "live" and measuring_from is not None and measured_elapsed >= options.duration:
                    completed = True
                    break
                if process.poll() is not None:
                    completed = options.source == "replay" and process.returncode == 0 and measuring_from is not None
                    if completed:
                        measured_elapsed = max((rows[-1][1] for rows in metrics.streams.values() if rows), default=0.)
                    if not completed:
                        error = "数据链进程提前退出: " + str(process.returncode)
                    break
                if now - last_snapshot >= .5:
                    last_snapshot = now
                    if (output / "samples.jsonl").stat().st_size > 512 * 1024**2:
                        raise RuntimeError("样本记录达到 512 MiB 上限")
                    free = shutil.disk_usage(output).free
                    if free < 25 * 1024**3:
                        raise RuntimeError("磁盘剩余空间低于 25 GiB 保留值")
                    if sum(p.stat().st_size for p in output.rglob("*") if p.is_file()) > 2 * 1024**3:
                        raise RuntimeError("本轮测试证据达到 2 GiB 上限")
                    probe_log = output / ("launch.log" if options.source == "live" else "replay/launch.log")
                    if probe_log.exists():
                        with probe_log.open(errors="replace") as reader:
                            reader.seek(log_position)
                            text = partial_line + reader.read()
                            log_position = reader.tell()
                        complete, _, partial_line = text.rpartition("\n")
                        visual_updates.feed(complete)
                        observed = [int(n) for n in re.findall(r"Retrieve (\d+) points from visual sparse map", complete)]
                        visual_frames += sum(n > 0 for n in observed)
                        visual_points_max = max([visual_points_max, *observed])
                    snapshot = metrics.summary(measured_elapsed)
                    snapshot["visual_constraints_positive_frames"] = visual_frames
                    snapshot["visual_update_diagnostics"] = visual_updates.summary()
                    verdict = evaluate(snapshot, standard, options.case, stationarity_confirmed=options.stationary_confirmed)
                    save(output / "state.json", {"phase": "measuring" if measuring_from else "warming_up",
                         "identity": identity, "metrics": snapshot, "verdict": verdict,
                         "cloud": cloud, "device_clock": device_clock,
                         "visual_constraints_points_max": visual_points_max,
                         "resources": {"load_1m": float(Path("/proc/loadavg").read_text().split()[0]),
                                       "disk_free_gib": free / 1024**3},
                         "image_age_s": max(0., now - preview_at["image"]),
                         "warmup_remaining_s": max(0., warmup - (now - ready_at)) if ready_at else warmup})
            samples.flush()
        except (Exception, KeyboardInterrupt) as exc:
            import traceback
            traceback.print_exc()
            error = str(exc) or "用户中断"
        finally:
            forced = stop(process)
            samples.close()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    log = (output / "launch.log").read_text(errors="replace")
    exits = replay_log_failures(log)
    errors = exits["child_exit_errors"] + exits["debugger_stop_signals"]
    if forced or error or tf_count[0]:
        errors.append(error or "forced_shutdown_or_unexpected_tf")
    replay_report = output / "replay/report.json"
    if options.source == "replay" and replay_report.exists():
        replay = json.loads(replay_report.read_text())
        if not replay["replay_chain_passed"]:
            errors.append("replay_driver_failed")
    hardware_errors = re.findall(r"Hardware Notification:[^\n]*", log)
    summary = metrics.summary(measured_elapsed)
    visual_log = output / ("launch.log" if options.source == "live" else "replay/launch.log")
    summary["visual_constraints_positive_frames"] = sum(int(n) > 0 for n in re.findall(
        r"Retrieve (\d+) points from visual sparse map", visual_log.read_text(errors="replace") if visual_log.exists() else ""))
    summary["visual_update_diagnostics"] = visual_update_summary(
        visual_log.read_text(errors="replace") if visual_log.exists() else "")
    fixture = (output / "return_fixture.request").exists()
    verdict = evaluate(summary, standard, options.case, finished=completed or bool(error), interrupted=stopping[0],
                       child_errors=errors, hardware_errors=hardware_errors, return_fixture_confirmed=fixture,
                       calibration_verified=not calibration.get("unverified", ["unknown"]),
                       stationarity_confirmed=options.stationary_confirmed)
    report = {"identity": identity, "metrics": summary, "verdict": verdict, "error": error,
              "child_errors": errors, "hardware_errors": hardware_errors, "forced_shutdown": forced,
              "tf_messages": tf_count[0], "finished_utc_ns": time.time_ns()}
    save(output / "report.json", report)
    rows = [f"# {options.case} SLAM 测试报告", "", f"标准：{standard['id']}（暂定工程目标）",
            f"结果：{verdict['status']}；独立精度认证：未完成", "",
            "| 检查项 | 实测 | 门限 | 结果 |", "| --- | --- | --- | --- |"]
    rows.extend(f"| {c['name']} | {c['value']} | {c['limit']} | {c['status']} |" for c in verdict["checks"])
    rows.extend(["", "硬件告警：", *hardware_errors, "", "机械外参/残余时差未验收时，不代表高精度或比赛就绪。"])
    (output / "report.md").write_text("\n".join(rows) + "\n", encoding="utf-8")
    save(output / "state.json", {"phase": "finished", "identity": identity, "metrics": summary,
                                  "verdict": verdict, "cloud": cloud, "device_clock": device_clock,
                                  "image_age_s": 0., "error": error, "hardware_errors": hardware_errors})
    print(json.dumps(verdict, ensure_ascii=False), flush=True)
    return 0 if verdict["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
