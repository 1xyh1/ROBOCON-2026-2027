"""Build-bound source identity and unreferenced LIVO replay diagnostics."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import re

import numpy as np


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def source_files(root: Path) -> dict[str, str]:
    paths = [root / "CMakeLists.txt", root / "package.xml"]
    for directory in ("src", "include"):
        paths.extend(p for p in (root / directory).rglob("*")
                     if p.is_file() and p.suffix in (".c", ".cc", ".cpp", ".cxx", ".h", ".hpp"))
    return {p.relative_to(root).as_posix(): sha256(p) for p in sorted(paths)}


def build_identity(executable: Path, checkout_markers: dict) -> dict:
    result = {"executable": str(executable), "executable_sha256": sha256(executable)}
    manifest_path = executable.with_name(executable.name + ".build.json")
    if not manifest_path.exists():
        # A marker in a mutable checkout is not evidence for an older binary.
        return {**result, "source_identity_verified": False, "source_identities": None,
                "unverified_reason": "legacy_binary_has_no_build_bound_source_manifest",
                "observed_checkout_markers_not_build_proof": checkout_markers}
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest["executable_sha256"] != result["executable_sha256"]:
        raise ValueError("LIVO executable does not match its build manifest")
    if source_files(Path(manifest["source_root"])) != manifest["source_files_sha256"]:
        raise ValueError("LIVO source snapshot changed after the recorded build")
    return {**result, "source_identity_verified": True,
            "source_identities": manifest["source_identities"],
            "build_manifest_sha256": sha256(manifest_path), "build_manifest": manifest}


def motion_summary(odometry) -> dict | None:
    """Estimated motion only; neither ATE nor accuracy without external truth."""
    if not odometry:
        return None
    rows = np.asarray(odometry, dtype=float)
    xyz, stamps = rows[:, 1:4], rows[:, 0]
    radius = np.linalg.norm(xyz - xyz[0], axis=1)
    steps, dt = np.linalg.norm(np.diff(xyz, axis=0), axis=1), np.diff(stamps)
    speeds = steps[dt > 0] / dt[dt > 0]
    return {"kind": "estimated_motion_without_independent_ground_truth",
            "path_length_m": float(steps.sum()), "max_radius_m": float(radius.max()),
            "end_displacement_m": float(radius[-1]),
            "max_step_m": float(steps.max()) if len(steps) else None,
            "max_linear_speed_mps": float(speeds.max()) if len(speeds) else None,
            "accuracy_evaluated": False}


def timing_summary(log: str) -> dict:
    cuts = re.findall(r"\[ROBOCON_CUT\].*?future_points=(\d+) max_future_ms=([-+\deE.]+)", log)
    propagation = re.findall(
        r"\[ROBOCON_PROP\] mode=(\d+) samples=(\d+) member=([-+\deE.]+) "
        r"begin=([-+\deE.]+) end=([-+\deE.]+) max_dt=([-+\deE.]+)", log)
    return {"instrumented": bool(cuts or propagation), "cut_groups": len(cuts),
            "groups_with_future_points": sum(int(n) > 0 for n, _ in cuts),
            "future_points_total": sum(int(n) for n, _ in cuts),
            "max_future_ms": max((float(ms) for _, ms in cuts), default=None),
            "propagation_groups": len(propagation),
            "max_absolute_integration_dt_s": max((abs(float(row[5])) for row in propagation), default=None),
            "initial_propagation": [dict(zip(("mode", "samples", "member", "begin", "end", "max_dt"), row))
                                    for row in propagation[:4]]}


class VisualUpdateAccumulator:
    """Consume complete log lines once, preserving the current visual frame."""
    pattern = re.compile(
        r"Retrieve \d+ points from visual sparse map|"
        r"\[ROBOCON_VIO\] level=(\d+) iteration=(\d+) measurements=(\d+) accepted=1 "
        r"hessian_trace=([-+\deE.]+) translation_step_m=([-+\deE.]+) rotation_step_rad=([-+\deE.]+)"
    )

    def __init__(self) -> None:
        self._instrumented = False
        self._iterations = self._frames = self._information_frames = self._measurements = 0
        self._frame_updated = self._frame_informed = False
        self._translation_max = self._rotation_max = None

    def feed(self, text: str) -> None:
        self._instrumented |= "[ROBOCON_VIO]" in text
        for match in self.pattern.finditer(text):
            if match.group(1) is None:
                self._frame_updated = self._frame_informed = False
                continue
            _, _, measurements, information, translation, rotation = match.groups()
            self._iterations += 1
            self._measurements += int(measurements)
            if not self._frame_updated:
                self._frames += 1
                self._frame_updated = True
            if float(information) > 0 and not self._frame_informed:
                self._information_frames += 1
                self._frame_informed = True
            self._translation_max = (float(translation) if self._translation_max is None
                                     else max(self._translation_max, float(translation)))
            self._rotation_max = (float(rotation) if self._rotation_max is None
                                  else max(self._rotation_max, float(rotation)))

    def summary(self) -> dict:
        return {
            "instrumented": self._instrumented,
            "accepted_update_iterations": self._iterations,
            "frames_with_accepted_update": self._frames,
            "frames_with_nonzero_image_information": self._information_frames,
            "accepted_pixel_measurements_over_iterations": self._measurements,
            "max_translation_step_m": self._translation_max,
            "max_rotation_step_rad": self._rotation_max,
        }


def visual_update_summary(log: str) -> dict:
    """Count traced EKF updates separately from retrieved feature candidates."""
    result = VisualUpdateAccumulator()
    result.feed(log)
    return result.summary()
