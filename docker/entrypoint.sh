#!/usr/bin/env bash
# Container entrypoint: put ROS 2 Humble (and the built workspace, if there
# is one) on the environment, then exec whatever was asked for.
#
# This runs for every entry into the container — `docker compose up`,
# `docker compose exec sim bash`, and any one-off `docker exec`. Sourcing
# here is what makes `ros2 launch` work in a fresh exec shell without any
# per-shell setup.
set -e

source /opt/ros/humble/setup.bash

WS_ROOT=/home/ubuntu/ws
if [ -f "${WS_ROOT}/install/setup.bash" ]; then
  source "${WS_ROOT}/install/setup.bash"
else
  echo "[entrypoint] ${WS_ROOT}/install/setup.bash not found —" \
       "run ./build.sh inside the container first."
fi

# Best-view frames written by the perception chain; created here so the
# first write does not race the directory into existence.
mkdir -p "${HOME}/.tb3_semantic_nav/evidence"

exec "$@"
