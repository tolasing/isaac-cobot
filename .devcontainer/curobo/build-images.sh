#!/usr/bin/env bash
# Builds isaac-cobot-base -> isaac-cobot-curobo before the devcontainer starts.
# Runs on the HOST via devcontainer initializeCommand.
# Pass --force to rebuild all images even if they already exist locally.
set -e

REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
ENV_FILE="${REPO_ROOT}/docker/.env.base"
CUROBO_ENV_FILE="${REPO_ROOT}/docker/.env.curobo"

FORCE=0
for arg in "$@"; do [[ "$arg" == "--force" ]] && FORCE=1; done

build_if_needed() {
    local image="$1"; shift
    if [[ "$FORCE" -eq 0 ]] && docker image inspect "${image}" > /dev/null 2>&1; then
        echo "[devcontainer] ${image} already exists, skipping build. (pass --force to rebuild)"
        return
    fi
    echo "[devcontainer] Building ${image}..."
    docker build "$@"
}

source "${ENV_FILE}"
source "${CUROBO_ENV_FILE}"

build_if_needed isaac-cobot-base \
    --network host \
    -f "${REPO_ROOT}/docker/Dockerfile.base" \
    --build-arg ISAACSIM_BASE_IMAGE_ARG="${ISAACSIM_BASE_IMAGE}" \
    --build-arg ISAACSIM_VERSION_ARG="${ISAACSIM_VERSION}" \
    --build-arg ISAACSIM_ROOT_PATH_ARG="${DOCKER_ISAACSIM_ROOT_PATH}" \
    --build-arg DOCKER_ISAAC_COBOT_PATH_ARG="${DOCKER_ISAAC_COBOT_PATH}" \
    --build-arg DOCKER_USER_HOME_ARG="${DOCKER_USER_HOME}" \
    -t isaac-cobot-base \
    "${REPO_ROOT}"

build_if_needed isaac-cobot-curobo \
    --network host \
    -f "${REPO_ROOT}/docker/Dockerfile.curobo" \
    --build-arg CUROBO_COMMIT_ARG="${CUROBO_COMMIT}" \
    --build-arg BT_CPP_COMMIT_ARG="${BT_CPP_COMMIT}" \
    -t isaac-cobot-curobo \
    "${REPO_ROOT}"

echo "[devcontainer] Images ready."

# X11 forwarding: generate a magic-cookie xauth file the compose file mounts
# into the container as XAUTHORITY. Best-effort -- don't fail the whole
# initializeCommand if there's no host X session (e.g. headless dev server).
touch /tmp/.docker.xauth
xauth nlist "${DISPLAY:-:0}" 2>/dev/null | sed -e 's/^..../ffff/' | xauth -f /tmp/.docker.xauth nmerge - 2>/dev/null || true
