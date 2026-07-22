#!/usr/bin/env bash
# Builds bt_bridge/'s pybind11 extension against Isaac Sim's own bundled Python interpreter and
# drops the resulting .so into scripts/mefron_lib/, so `from mefron_lib import bt_bridge` (see
# scripts/mefron_lib/behavior_tree.py) resolves it as an ordinary package submodule.
#
# Run this once per container after `docker compose up` (isaac-cobot-curobo profile) -- bt_bridge/
# is part of the live, bind-mounted repo, not baked into the image (see docker/Dockerfile.curobo's
# own comment on why BehaviorTree.CPP/pybind11 themselves ARE baked in but this extension isn't),
# so it has to be (re)built whenever bt_bridge/src/bt_bridge.cpp changes. See
# docs/behavior-tree-migration.md for the full design.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${REPO_ROOT}/bt_bridge/build"
ISAACSIM_ROOT_PATH="${ISAACSIM_ROOT_PATH:-/isaac-sim}"
PYTHON_EXECUTABLE="${ISAACSIM_ROOT_PATH}/kit/python/bin/python3"

if [ ! -x "${PYTHON_EXECUTABLE}" ]; then
    echo "error: ${PYTHON_EXECUTABLE} not found -- set ISAACSIM_ROOT_PATH if Isaac Sim isn't at /isaac-sim" >&2
    exit 1
fi

cmake -S "${REPO_ROOT}/bt_bridge" -B "${BUILD_DIR}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DPython3_EXECUTABLE="${PYTHON_EXECUTABLE}"
cmake --build "${BUILD_DIR}" --parallel "$(nproc)"

built_so=$(find "${BUILD_DIR}" -maxdepth 1 -name 'mefron_bt_bridge*.so' -print -quit)
if [ -z "${built_so}" ]; then
    echo "error: build succeeded but no mefron_bt_bridge*.so found in ${BUILD_DIR}" >&2
    exit 1
fi
cp -f "${built_so}" "${REPO_ROOT}/scripts/mefron_lib/"
echo "Installed $(basename "${built_so}") -> scripts/mefron_lib/"
