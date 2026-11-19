#!/usr/bin/env bash
# Build the isolated FAST-LIVO2 overlay using the workspace sensors and Sophus.
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
source env.sh
export CMAKE_PREFIX_PATH="$robocon_workspace/resources/runtime/fusion-deps:$CMAKE_PREFIX_PATH"
export CMAKE_BUILD_PARALLEL_LEVEL="${FUSION_BUILD_JOBS:-2}"
export MAKEFLAGS="-j$CMAKE_BUILD_PARALLEL_LEVEL -l$CMAKE_BUILD_PARALLEL_LEVEL"
mkdir -p resources/state/fast_livo/Log/{Debug,result,image,pcd}
colcon --log-base log/experimental build --paths src/experimental/vikit_common src/experimental/fast_livo \
  --build-base build/experimental --install-base install/experimental --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
python3 - <<'PY'
import json
from pathlib import Path
from robocon_core.livo_evidence import source_files, sha256
root = Path.cwd()
source = root / "src/experimental/fast_livo"
exe = root / "install/experimental/fast_livo/lib/fast_livo/fastlivo_mapping"
manifest = {
    "source_root": str(source), "source_files_sha256": source_files(source),
    "executable_sha256": sha256(exe), "source_identities": {
        "FAST-LIVO2-ROS2": json.loads((source / ".robocon-source.json").read_text()),
        "vikit_common_source": source_files(root / "src/experimental/vikit_common"),
        "SophusConfig_sha256": sha256(root / "resources/runtime/fusion-deps/share/sophus/cmake/SophusConfig.cmake"),
    },
}
exe.with_name(exe.name + ".build.json").write_text(json.dumps(manifest, indent=2) + "\n")
PY
