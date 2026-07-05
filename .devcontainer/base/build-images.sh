#!/usr/bin/env bash
# Builds isaac-cobot-base before the devcontainer starts.
# Runs on the HOST via devcontainer initializeCommand.
set -e

REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
ENV_FILE="${REPO_ROOT}/docker/.env.base"

source "${ENV_FILE}"

# Tagged with the Isaac Sim version pin (dots stripped), not a bare name --
# docker image tags are global on this machine, NOT git-branch-scoped, and
# this branch may be pinned to a different Isaac Sim version than main (see
# docker/.env.base). Without this, this script's unconditional rebuild
# below would silently overwrite whatever image the bare tag pointed to,
# even one belonging to a different branch's devcontainer. Keep in sync
# with ISAACSIM_VERSION if that's bumped again.
IMAGE_SUFFIX="-${ISAACSIM_VERSION//./}"

echo "[devcontainer] Building isaac-cobot-base${IMAGE_SUFFIX}..."
docker build \
    --network host \
    -f "${REPO_ROOT}/docker/Dockerfile.base" \
    --build-arg ISAACSIM_BASE_IMAGE_ARG="${ISAACSIM_BASE_IMAGE}" \
    --build-arg ISAACSIM_VERSION_ARG="${ISAACSIM_VERSION}" \
    --build-arg ISAACSIM_ROOT_PATH_ARG="${DOCKER_ISAACSIM_ROOT_PATH}" \
    --build-arg DOCKER_ISAAC_COBOT_PATH_ARG="${DOCKER_ISAAC_COBOT_PATH}" \
    --build-arg DOCKER_USER_HOME_ARG="${DOCKER_USER_HOME}" \
    -t "isaac-cobot-base${IMAGE_SUFFIX}" \
    "${REPO_ROOT}"

echo "[devcontainer] Image ready."

# X11 forwarding: generate a magic-cookie xauth file the compose file mounts
# into the container as XAUTHORITY. Best-effort -- don't fail the whole
# initializeCommand if there's no host X session (e.g. headless dev server).
touch /tmp/.docker.xauth
xauth nlist "${DISPLAY:-:0}" 2>/dev/null | sed -e 's/^..../ffff/' | xauth -f /tmp/.docker.xauth nmerge - 2>/dev/null || true
