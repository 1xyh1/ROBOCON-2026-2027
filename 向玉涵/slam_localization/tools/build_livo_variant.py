"""Snapshot and build one LIVO ablation into a new, isolated colcon overlay.

Run in the mini-host Humble/venv environment with install-livo already sourced.
The base installed executable and its source checkout are never replaced.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

from robocon_core.livo_evidence import source_files, sha256

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path, help="Must be a new directory")
    parser.add_argument("--patch", type=Path, action="append", default=[])
    args = parser.parse_args()
    if sys.platform != "linux" or os.environ.get("ROS_DISTRO") != "humble":
        parser.error("run on the Ubuntu 22.04 / ROS 2 Humble mini host")
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    source = output / "source"
    shutil.copytree(args.source.resolve(), source,
                    ignore=shutil.ignore_patterns(".git", "Log", "__pycache__"))
    (source / "Log/Debug").mkdir(parents=True)
    environment = dict(os.environ, GIT_CEILING_DIRECTORIES=str(source.parent))
    applied = []
    for patch in args.patch:
        patch = patch.resolve()
        subprocess.run(["git", "apply", "--check", str(patch)], cwd=source, env=environment, check=True)
        subprocess.run(["git", "apply", str(patch)], cwd=source, env=environment, check=True)
        applied.append({"path": str(patch), "sha256": sha256(patch)})
    before = source_files(source)
    environment.update(CMAKE_BUILD_PARALLEL_LEVEL="2", MAKEFLAGS="-j2 -l2")
    environment["CMAKE_PREFIX_PATH"] = str(ROOT / "resources/runtime/fusion-deps") + ":" + environment.get("CMAKE_PREFIX_PATH", "")
    subprocess.run([
        sys.executable, shutil.which("colcon"), "--log-base", str(output / "log"), "build",
        "--base-paths", str(source), "--build-base", str(output / "build"),
        "--install-base", str(output / "install"), "--packages-select", "fast_livo",
        "--allow-overriding", "fast_livo", "--executor", "sequential",
        "--event-handlers", "console_direct+", "--cmake-args", "-DBUILD_TESTING=OFF",
        "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON"], cwd=ROOT, env=environment, check=True)
    if source_files(source) != before:
        raise RuntimeError("source snapshot changed during build")
    executable = output / "install/fast_livo/lib/fast_livo/fastlivo_mapping"
    manifest = {
        "schema_version": 1, "source_root": str(source), "source_files_sha256": before,
        "executable_sha256": sha256(executable), "additional_patches": applied,
        "source_identities": {
            "FAST-LIVO2-ROS2": json.loads((source / ".robocon-source.json").read_text()),
            "vikit_common_source": source_files(ROOT / "src/experimental/vikit_common"),
            "SophusConfig_sha256": sha256(ROOT / "resources/runtime/fusion-deps/share/sophus/cmake/SophusConfig.cmake")},
        "cmake_cache_sha256": sha256(output / "build/fast_livo/CMakeCache.txt"),
        "compile_commands_sha256": sha256(output / "build/fast_livo/compile_commands.json"),
        "compiler": subprocess.check_output(["c++", "--version"], text=True).splitlines()[0]}
    destination = executable.with_name(executable.name + ".build.json")
    destination.write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps({"executable": str(executable), "manifest": str(destination),
                      "sha256": manifest["executable_sha256"]}), flush=True)


if __name__ == "__main__":
    main()
