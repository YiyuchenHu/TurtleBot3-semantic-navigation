#!/usr/bin/env bash
#
# acceptance_run.sh -- one-command end-to-end acceptance run.
#
#   ./scripts/acceptance_run.sh                       # three-target acceptance
#   ./scripts/acceptance_run.sh --world warehouse_models_person
#   ./scripts/acceptance_run.sh --dry-run             # plan only, nothing built
#   ./scripts/acceptance_run.sh --help
#
# This wrapper exists so the user runs ONE command. Its only job is to prepare
# the environment exactly as the README tells every terminal to prepare it, and
# then hand over to scripts/acceptance_run.py, which does the real work.
#
# WHY THE WORK IS IN PYTHON, NOT HERE
#   The run has to do OccupancyGrid arithmetic over tens of thousands of int8
#   cells, reach into vision_msgs/Detection3DArray fields, subscribe to three
#   topics concurrently with independent timeouts, do float geometry against
#   Gazebo ground truth and print a scored table. In bash that is a pile of
#   `ros2 topic echo | grep | awk` that breaks the first time a YAML block
#   re-flows; rclpy does it directly. The shell layer is only start-up glue.
#   Read the header of acceptance_run.py for the longer version.
#
# All arguments are passed straight through to the Python driver.

set -euo pipefail

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

# --help and --dry-run must work on a machine where nothing is sourced or built
# yet -- the driver imports ROS lazily for exactly that reason -- so they are
# answered before any of the environment checks below.
for arg in "$@"; do
    case "$arg" in
        -h|--help|--dry-run)
            exec python3 "$REPO_ROOT/scripts/acceptance_run.py" "$@"
            ;;
    esac
done

if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo "acceptance_run: /opt/ros/humble/setup.bash not found -- this course " \
         "needs ROS 2 Humble." >&2
    exit 2
fi
if [ ! -f "$REPO_ROOT/install/setup.bash" ]; then
    echo "acceptance_run: $REPO_ROOT/install/setup.bash not found -- build the" \
         "workspace first:" >&2
    echo "    cd $REPO_ROOT && source /opt/ros/humble/setup.bash && colcon build --symlink-install" >&2
    exit 2
fi

# The same four lines the README puts at the top of every terminal. The tmux
# windows repeat them for themselves; this is for the driver process, which
# needs rclpy and the course message packages.
#
# `set -u` is lifted across the two sources: ROS's own setup.bash reads
# AMENT_TRACE_SETUP_FILES (and friends) without defaulting them, so under -u it
# aborts with "AMENT_TRACE_SETUP_FILES: unbound variable" before it exports a
# single path. Restore -u immediately afterwards so the rest of the script keeps
# the strict behaviour.
cd "$REPO_ROOT"
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$REPO_ROOT/install/setup.bash"
set -u
export TURTLEBOT3_MODEL=waffle_pi

# exec: the driver becomes this process, so its exit code is the script's exit
# code and Ctrl-C reaches it directly rather than orphaning it behind a wrapper.
exec python3 "$REPO_ROOT/scripts/acceptance_run.py" "$@"
