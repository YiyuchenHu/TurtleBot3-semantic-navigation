#!/usr/bin/env python3
"""
smoke_test_query.py — Lightweight Stage-4 semantic query integration test.

Publishes a series of text commands to /semantic_query_node/command,
waits for each SemanticQueryResult response, and reports pass/fail.

Requires: semantic_query_node + semantic_memory_node already running.

Usage:
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    python3 src/tb3_query/test/smoke_test_query.py
"""

from __future__ import annotations

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String

try:
    from tb3_query.msg import SemanticQueryResult
except ImportError:
    print("ERROR: tb3_query messages not found. Did you source install/setup.bash?")
    sys.exit(1)


TIMEOUT_SEC = 5.0

# The three live targets come from src/tb3_bringup/config/semantic_targets.yaml
# (person / trash_can / chair). `table` and `stop_sign` were retired when the
# target set was cut to three, so the cases that used to assert them had
# inverted: they demanded success from two targets the registry no longer
# resolves, and demanded failure from `chair`, which is now a live target.
TEST_CASES: list[dict] = [
    # Default (no index) — matches whichever instance is nearest.
    {"command": "go to the person",      "expect_success": True,  "expect_name": "person"},
    # "trash can" resolves through _PHRASE_ALIASES to the registry entry whose
    # detector_label is read from semantic_targets.yaml, never hard-coded.
    {"command": "go to the trash can",   "expect_success": True,  "expect_name": "trash_can"},
    {"command": "go to the chair",       "expect_success": True,  "expect_name": "chair"},
    # Retired targets must now be rejected, not resolved.
    {"command": "go to the table",       "expect_success": False, "expect_name": ""},
    {"command": "go to the stop sign",   "expect_success": False, "expect_name": ""},
    # Never was a target.
    {"command": "go to the fridge",      "expect_success": False, "expect_name": ""},

    # Indexed lookups exercising the three accepted spellings of
    # "<target> <integer>". Success depends on whether the requested
    # `<label>_<N>` already exists in semantic memory at the time of
    # the request, so all three are marked optional (the harness reports
    # rejection as a PASS when expect_success=False).
    # If you have warehouse_models_person running and the bot has
    # observed the corresponding person, flip these to expect_success=True.
    {"command": "go to person 3",        "expect_success": False, "expect_name": "person"},
    {"command": "go to person_2",        "expect_success": False, "expect_name": "person"},
    {"command": "go to person number 4", "expect_success": False, "expect_name": "person"},
]


class SmokeTestNode(Node):

    def __init__(self) -> None:
        super().__init__("smoke_test_query")

        qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._cmd_pub = self.create_publisher(
            String, "/semantic_query_node/command", qos
        )
        self._result: SemanticQueryResult | None = None
        self.create_subscription(
            SemanticQueryResult,
            "/semantic_query_node/selected_target",
            self._result_cb,
            qos,
        )

    def _result_cb(self, msg: SemanticQueryResult) -> None:
        self._result = msg

    def send_and_wait(self, command: str) -> SemanticQueryResult | None:
        self._result = None
        msg = String()
        msg.data = command
        self._cmd_pub.publish(msg)

        deadline = time.time() + TIMEOUT_SEC
        while self._result is None and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._result


def main() -> None:
    rclpy.init()
    node = SmokeTestNode()

    time.sleep(1.0)

    passed = 0
    failed = 0
    total = len(TEST_CASES)

    print("=" * 60)
    print("  tb3_query smoke test  (%d cases)" % total)
    print("=" * 60)

    for i, tc in enumerate(TEST_CASES, 1):
        cmd = tc["command"]
        expect_ok = tc["expect_success"]
        expect_name = tc["expect_name"]

        print("\n[%d/%d] Command: '%s'" % (i, total, cmd))
        result = node.send_and_wait(cmd)

        if result is None:
            print("  FAIL — no response within %.1fs" % TIMEOUT_SEC)
            failed += 1
            continue

        ok = True

        if result.success != expect_ok:
            print("  FAIL — success=%s, expected %s" % (result.success, expect_ok))
            ok = False

        if expect_ok and expect_name and result.semantic_name != expect_name:
            print("  FAIL — semantic_name='%s', expected '%s'"
                  % (result.semantic_name, expect_name))
            ok = False

        if ok:
            if result.success:
                print("  PASS — %s → %s at (%.2f, %.2f)  conf=%.2f"
                      % (result.semantic_name, result.object_id,
                         result.position.x, result.position.y,
                         result.confidence))
            else:
                print("  PASS — correctly rejected: %s" % result.status_message)
            passed += 1
        else:
            print("  status_message: %s" % result.status_message)
            failed += 1

    print("\n" + "=" * 60)
    print("  Results: %d passed, %d failed, %d total" % (passed, failed, total))
    print("=" * 60)

    node.destroy_node()
    rclpy.shutdown()

    sys.exit(0 if failed == 0 else 1)


if __name__ == "__main__":
    main()
