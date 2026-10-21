#!/usr/bin/env bash
# Build ROS2 workspace (Humble). Uses system Python to avoid conda 'em' module errors.

set -e

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${WS_ROOT}"

echo "=== Building ROS2 workspace ==="
echo "Workspace: ${WS_ROOT}"
echo ""

# Force system Python (avoids conda 'em' module error; overrides CMake cache)
export PATH="/usr/bin:${PATH}"
export Python3_EXECUTABLE="/usr/bin/python3"

source /opt/ros/humble/setup.bash

# Marker used to detect that the workspace has been built with system Python
# (kept for parity with the original project; currently no msg packages need
# a forced reconfigure, but the marker is still written on success).
MARKER="${WS_ROOT}/.build_used_system_python"

colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 "$@"
result=$?

echo ""
if [ ${result} -ne 0 ]; then
  echo "=== Build FAILED (exit code ${result}) ==="
  exit ${result}
fi

touch "${MARKER}" 2>/dev/null || true

# Inside the sim container: generate the coloured-shirt person models the
# default LocateAnything world uses (git-ignored, ~28 MB each), once.
if [ -d /opt/gazebo_models/person_standing ] \
   && [ ! -d "${WS_ROOT}/src/tb3_frontier_exploration/models/person_green_shirt" ]; then
  echo "=== Generating person_{red,green,blue}_shirt Gazebo models ==="
  python3 "${WS_ROOT}/scripts/make_person_variants.py" \
    --dst "${WS_ROOT}/src/tb3_frontier_exploration/models"
fi

echo "=== Build SUCCESS ==="
echo "Built packages:"
for d in "${WS_ROOT}/install/"*/; do
  name=$(basename "$d")
  [ "$name" = "COLCON_IGNORE" ] && continue
  echo "  - ${name}"
done
echo "  (source: ${WS_ROOT}/install/setup.bash)"
echo ""
