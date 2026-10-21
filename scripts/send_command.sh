#!/usr/bin/env bash
# send_command.sh — publish /user_command with receipt verification.
# Run on the HOST; it exec's into the tb3-semantic-nav-sim-1 container.
#
# `ros2 topic pub --once` intermittently loses the first message (known
# since the August runbook). Instead of blind double-sends (which the
# preemption path would treat as two commands), publish once, then check
# the stack log for the coordinator's "User command:" receipt; retry up
# to 3 times. LOG is the host path of the file the stack launch is
# tee'ing into (see docs/la_runbook.md §4).
#
# Usage: ./scripts/send_command.sh "go to the trash can" /path/to/stack.log
set -u
CMD="$1"
LOG="$2"
for attempt in 1 2 3; do
  BEFORE=$(grep -c "User command: '$CMD'" "$LOG" 2>/dev/null | head -1)
  BEFORE=${BEFORE:-0}
  docker exec tb3-semantic-nav-sim-1 bash -c \
    "source /opt/ros/humble/setup.bash && source /home/ubuntu/ws/install/setup.bash && \
     ros2 topic pub --once /user_command std_msgs/String \"data: '$CMD'\"" \
     >/dev/null 2>&1
  for i in $(seq 1 10); do
    sleep 0.5
    AFTER=$(grep -c "User command: '$CMD'" "$LOG" 2>/dev/null | head -1)
    AFTER=${AFTER:-0}
    if [ "$AFTER" -gt "$BEFORE" ]; then
      echo "RECEIVED (attempt $attempt): $CMD"
      exit 0
    fi
  done
  echo "attempt $attempt not received, retrying..."
done
echo "FAILED to deliver: $CMD"
exit 1
