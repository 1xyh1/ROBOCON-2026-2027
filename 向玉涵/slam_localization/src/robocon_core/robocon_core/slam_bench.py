"""ROS/GUI-independent SLAM bench measurements and versioned verdicts.

Static spread, drift, closure and absolute accuracy are different quantities.
No ground truth is synthesized from the estimator's own output.
"""
from __future__ import annotations

from collections import defaultdict
import json
import math
from pathlib import Path
import struct
import numpy as np


def stamp_from_cdr(payload):
    """ROS Humble CDR Header stamp without expanding 20k CustomPoint objects."""
    if len(payload) < 12 or bytes(payload[:2]) not in (b"\x00\x01", b"\x00\x00"):
        raise ValueError("expected CDR1 encapsulation and a leading ROS Header")
    seconds, nanos = struct.unpack_from("<iI" if payload[1] == 1 else ">iI", payload, 4)
    if nanos >= 1_000_000_000:
        raise ValueError("invalid ROS nanoseconds")
    return seconds + nanos / 1e9


def quaternion_yaw(q):
    q = np.asarray(q, dtype=float)
    q /= np.linalg.norm(q)
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def xyz_from_cloud(data, fields, point_step, row_step, width, height, bigendian=False, limit=3000):
    """Read real PointCloud2 XYZ respecting fields, row padding and byte order."""
    if not width or not height:
        return np.empty((0, 3))
    selected = {field.name: field for field in fields if field.datatype in (7, 8) and field.count == 1}
    if not all(key in selected for key in ("x", "y", "z")):
        raise ValueError("XYZ float32/float64 fields required")
    dtype = np.dtype({"names": ["x", "y", "z"],
                      "formats": [(">" if bigendian else "<") + ("f4" if selected[key].datatype == 7 else "f8")
                                  for key in ("x", "y", "z")],
                      "offsets": [selected[key].offset for key in ("x", "y", "z")], "itemsize": point_step})
    points = np.ndarray((height, width), dtype=dtype, buffer=data, strides=(row_step, point_step)).reshape(-1)
    points = points[::max(1, math.ceil(len(points) / limit))]
    xyz = np.column_stack([points[key] for key in ("x", "y", "z")])
    return xyz[np.isfinite(xyz).all(axis=1)]


class BenchMetrics:
    def __init__(self):
        self.streams = defaultdict(list)
        self.poses = []
        self.health = defaultdict(list)
        self.invalid = 0

    def add(self, stream, stamp, elapsed, age=None, pose=None):
        values = [stamp, elapsed] + ([] if age is None else [age]) + ([] if pose is None else list(pose))
        if (not all(math.isfinite(v) for v in values) or stamp <= 0
                or (pose is not None and not .99 <= np.linalg.norm(pose[3:]) <= 1.01)):
            self.invalid += 1
            return
        self.streams[stream].append((stamp, elapsed, age))
        if pose is not None:
            self.poses.append((stamp, elapsed, *pose))

    def add_health(self, sensor, elapsed, valid):
        self.health[sensor].append((elapsed, bool(valid)))

    def summary(self, elapsed):
        result = {"elapsed_s": elapsed, "streams": {}, "invalid_samples": self.invalid,
                  "static": {}, "trajectory": [], "health": {}}
        for sensor, rows in self.health.items():
            result["health"][sensor] = {
                "count": len(rows), "rejected": sum(not valid for _, valid in rows),
                "max_receive_gap_s": float(np.diff([0., *[r[0] for r in rows], max(elapsed, rows[-1][0])]).max())}
        for name, rows in self.streams.items():
            stamps = np.asarray([row[0] for row in rows])
            arrivals = np.asarray([row[1] for row in rows])
            ages = [row[2] for row in rows if row[2] is not None]
            deltas = np.diff(stamps)
            span = stamps[-1] - stamps[0] if len(stamps) > 1 else 0.
            receive_gaps = np.diff(np.r_[0., arrivals, max(elapsed, arrivals[-1])])
            result["streams"][name] = {
                "count": len(rows), "hz": (len(rows) - 1) / span if span > 0 else 0.,
                "coverage_s": float(arrivals[-1] - arrivals[0]),
                "max_stamp_gap_s": float(deltas.max()) if len(deltas) else None,
                "max_receive_gap_s": float(receive_gaps.max()),
                "last_receive_age_s": max(0., elapsed - float(arrivals[-1])),
                "non_increasing": int((deltas <= 0).sum()),
                "age_p99_s": float(np.percentile(ages, 99)) if ages else None,
                "age_min_s": float(min(ages)) if ages else None,
            }
        if self.poses:
            poses = np.asarray(self.poses)
            xyz = poses[:, 2:5]
            yaw = np.unwrap([quaternion_yaw(row[5:9]) for row in poses])
            center = np.median(xyz, axis=0)
            deviation = xyz - center
            first = poses[:, 0] <= poses[0, 0] + 1.
            last = poses[:, 0] >= poses[-1, 0] - 1.
            delta_yaw = float(np.median(yaw[last]) - np.median(yaw[first]))
            result["static"] = {
                "horizontal_p95_m": float(np.percentile(np.linalg.norm(deviation[:, :2], axis=1), 95)),
                "vertical_p95_m": float(np.percentile(abs(deviation[:, 2]), 95)),
                "yaw_p95_deg": float(np.percentile(abs(yaw - np.median(yaw)), 95) * 180 / np.pi),
                "max_excursion_m": float(np.linalg.norm(xyz - np.median(xyz[first], axis=0), axis=1).max()),
                "closure_translation_m": float(np.linalg.norm(np.median(xyz[last], axis=0) - np.median(xyz[first], axis=0))),
                "closure_yaw_deg": abs(math.degrees(math.atan2(math.sin(delta_yaw), math.cos(delta_yaw)))),
            }
            stride = max(1, math.ceil(len(xyz) / 3000))
            result["trajectory"] = xyz[::stride].tolist()
            result["last_pose"] = poses[-1, 2:].tolist()
        return result


def evaluate(summary, standard, case_id, *, finished=False, interrupted=False, child_errors=(),
             hardware_errors=(), return_fixture_confirmed=False, calibration_verified=False,
             stationarity_confirmed=False):
    case = standard["cases"][case_id]
    runtime = standard["runtime"]
    checks = []

    def check(name, value, limit, passed, evidence=True):
        checks.append({"name": name, "value": value, "limit": limit,
                       "status": ("PASS" if passed else "FAIL") if evidence else "NEEDS_EVIDENCE"})

    duration = summary["elapsed_s"]
    check("有效测试时长", duration, case["minimum_s"], duration >= case["minimum_s"] * .995)
    check("异常退出/无效位姿", len(child_errors) + summary["invalid_samples"], 0,
          not child_errors and summary["invalid_samples"] == 0)
    visual = summary.get("visual_update_diagnostics", {})
    information = visual.get("frames_with_nonzero_image_information", 0)
    check("实际视觉更新帧（非零图像信息）", information if visual.get("instrumented") else None,
          "> 0", information > 0, evidence=visual.get("instrumented", False))
    for sensor in runtime["required_health"]:
        health = summary.get("health", {}).get(sensor, {})
        check(sensor + " 持续时间健康", health.get("rejected"), 0,
              health.get("count", 0) >= 2 and health.get("rejected") == 0
              and health["max_receive_gap_s"] <= runtime["max_health_gap_s"])
    for name, minimum in {"odom": runtime["min_odom_hz"], **runtime["min_input_hz"]}.items():
        stream = summary["streams"].get(name, {})
        present = stream.get("count", 0) >= 2
        check(name + " 频率 Hz", stream.get("hz"), minimum, present and stream["hz"] >= minimum)
        check(name + " 覆盖比例", stream.get("coverage_s", 0.) / max(duration, .001),
              runtime["min_coverage_fraction"], present and stream["coverage_s"] >= duration * runtime["min_coverage_fraction"])
        check(name + " 倒退/重复时戳", stream.get("non_increasing"), 0,
              present and stream["non_increasing"] == 0)
    odom = summary["streams"].get("odom", {})
    for key, label, limit in (("max_stamp_gap_s", "里程计测量最大间断 s", runtime["max_odom_gap_s"]),
                              ("max_receive_gap_s", "里程计接收最大间断 s（含首尾）", runtime["max_odom_gap_s"]),
                              ("age_p99_s", "里程计消息龄 P99 s", runtime["max_age_p99_s"])):
        value = odom.get(key)
        check(label, value, limit, value is not None and value <= limit)
    value = odom.get("age_min_s")
    check("未来时间门限 s", value, runtime["min_age_s"], value is not None and value >= runtime["min_age_s"])
    if case_id == "SB00":
        check("硬件启动错误", len(hardware_errors), 0, not hardware_errors)
    if case_id == "SB01":
        check("设备固定/未触碰的工况确认", stationarity_confirmed, True, stationarity_confirmed,
              evidence=stationarity_confirmed)
        labels = {"horizontal_p95_m": "水平位置 P95 m", "vertical_p95_m": "高度位置 P95 m",
                  "yaw_p95_deg": "航向 P95 °", "max_excursion_m": "相对初始最大偏移 m"}
        for key, limit in standard["static"].items():
            value = summary["static"].get(key)
            check(labels[key], value, limit, value is not None and value <= limit,
                  evidence=stationarity_confirmed)
    if case_id == "SB02":
        check("回到同一定位夹具的现场证据", return_fixture_confirmed, True,
              return_fixture_confirmed, evidence=return_fixture_confirmed)
        excursion = summary["static"].get("max_excursion_m", 0.)
        check("路径离开起点 m", excursion, standard["closure"]["min_excursion_m"],
              excursion >= standard["closure"]["min_excursion_m"])
        for key, limit in (("closure_translation_m", standard["closure"]["translation_m"]),
                           ("closure_yaw_deg", standard["closure"]["yaw_deg"])):
            value = summary["static"].get(key)
            check(key, value, limit, value is not None and value <= limit,
                  evidence=return_fixture_confirmed)
    if case_id in ("SB03", "SB04", "SB05"):
        check("独立真值/标定/故障恢复证据尚未接入自动验收", None, "待专项证据", False, evidence=False)
    if interrupted:
        status = "INTERRUPTED"
    elif not finished:
        status = "RUNNING"
    elif any(c["status"] == "FAIL" for c in checks):
        status = "FAIL"
    elif any(c["status"] == "NEEDS_EVIDENCE" for c in checks):
        status = "NEEDS_EVIDENCE"
    else:
        status = "PASS"
    return {"standard_id": standard["id"], "case_id": case_id, "status": status, "checks": checks,
            "accuracy_certified": False, "calibration_verified": calibration_verified,
            "scope": "engineering_test_only_not_competition_approval"}


def load_bench_view(path: Path) -> dict:
    """Read a bench snapshot/report or an LIO/LIVO replay report for review.

    Preserve recorded verdicts: opening an old report never re-evaluates it
    using today's standard. Replay-chain results have no accuracy verdict.
    """
    document = json.loads(path.read_text(encoding="utf-8"))
    if "metrics" in document and "identity" in document:
        if "phase" in document:
            return document
        return {**document, "phase": "finished"}
    if "replay_chain_passed" not in document:
        raise ValueError("请选择测试台 state/report 或 FAST-LIO/LIVO 回放 report.json")

    trajectory_path = path.parent / "odometry.json"
    trajectory = []
    if trajectory_path.exists():
        rows = json.loads(trajectory_path.read_text(encoding="utf-8"))
        stride = max(1, math.ceil(len(rows) / 3000))
        trajectory = [row[1:4] for row in rows[::stride]]
    visual = document.get("visual_update_diagnostics", {})
    motion = document.get("estimated_motion") or {}
    build = document.get("build_identity", {})
    algorithm = "FAST-LIVO2" if "build_identity" in document else "FAST-LIO"
    identity = {"case_id": "REPLAY", "source": "replay", "algorithm": algorithm,
                "executable_sha256": build.get("executable_sha256", document.get("executable_sha256")),
                "calibration": document.get("calibration_evidence", {"unverified": ["not_recorded"]})}
    checks = [{"name": "回放数据链（不代表定位通过）", "value": document["replay_chain_passed"],
               "limit": True, "status": "PASS" if document["replay_chain_passed"] else "FAIL"},
              {"name": "独立真值精度验收", "value": None, "limit": "待独立真值", "status": "NEEDS_EVIDENCE"},
              {"name": "估计最大半径 m（不是误差）", "value": motion.get("max_radius_m"),
               "limit": "诊断值，无通过门限", "status": "NEEDS_EVIDENCE"},
              {"name": "实际视觉更新帧", "value": visual.get("frames_with_nonzero_image_information"),
               "limit": "诊断值", "status": "NEEDS_EVIDENCE"},
              {"name": "轨迹原始文件", "value": trajectory_path.name if trajectory_path.exists() else None,
               "limit": "odometry.json", "status": "PASS" if trajectory else "NEEDS_EVIDENCE"}]
    metrics = {"elapsed_s": document.get("odometry_measurement_span_s", 0), "static": {},
               "trajectory": trajectory, "estimated_motion": motion,
               "streams": {"odom": {"hz": document.get("odometry_measurement_hz"),
                    "age_p99_s": document.get("sim_clock_age_p99_s"),
                    "max_receive_gap_s": document.get("max_odometry_gap_s")}}}
    return {"phase": "finished", "identity": identity, "metrics": metrics, "error": document.get("error", ""),
            "verdict": {"status": "NEEDS_EVIDENCE" if document["replay_chain_passed"] else "FAIL",
                        "accuracy_certified": False, "checks": checks},
            "review_scope": "replay_diagnostics_not_localization_acceptance"}
