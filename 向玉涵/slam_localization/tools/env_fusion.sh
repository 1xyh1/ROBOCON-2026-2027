#!/usr/bin/env bash
source "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)/env.sh"
export CMAKE_PREFIX_PATH="$robocon_workspace/resources/runtime/fusion-deps:$CMAKE_PREFIX_PATH"
source "$robocon_workspace/install/experimental/local_setup.bash"
