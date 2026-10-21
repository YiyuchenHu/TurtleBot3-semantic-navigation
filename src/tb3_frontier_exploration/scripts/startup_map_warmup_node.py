#!/usr/bin/env python3
"""
One-shot startup scan: pure rotation on cmd_vel to seed SLAM / frontier quality before exploration.

Sequence: delay → turn left ~45° → pause → return to heading → pause → turn right ~45° → pause → return.
Then publishes latched std_msgs/Bool true on exploration_warmup_complete (or param topic).
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class StartupMapWarmupNode(Node):
    _PH_INIT_WAIT = 0
    _PH_TURN_LEFT = 1
    _PH_PAUSE_1 = 2
    _PH_RETURN_1 = 3
    _PH_PAUSE_2 = 4
    _PH_TURN_RIGHT = 5
    _PH_PAUSE_3 = 6
    _PH_RETURN_2 = 7
    _PH_DONE = 8

    def __init__(self):
        super().__init__("startup_map_warmup_node")
        self.declare_parameter("cmd_vel_topic", "cmd_vel")
        self.declare_parameter("warmup_complete_topic", "exploration_warmup_complete")
        self.declare_parameter("startup_delay_sec", 3.0)
        self.declare_parameter("pause_between_sec", 0.5)
        self.declare_parameter("scan_angle_deg", 45.0)
        self.declare_parameter("scan_angular_speed", 0.4)
        self.declare_parameter("timer_rate_hz", 50.0)

        self._cmd_topic = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        self._complete_topic = (
            self.get_parameter("warmup_complete_topic").get_parameter_value().string_value
        )
        angle_rad = math.radians(
            float(self.get_parameter("scan_angle_deg").get_parameter_value().double_value)
        )
        self._omega = float(self.get_parameter("scan_angular_speed").get_parameter_value().double_value)
        if self._omega <= 0.0:
            self._omega = 0.4
        self._turn_duration = angle_rad / self._omega
        self._startup_delay = float(
            self.get_parameter("startup_delay_sec").get_parameter_value().double_value
        )
        self._pause = float(self.get_parameter("pause_between_sec").get_parameter_value().double_value)

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._pub_cmd = self.create_publisher(Twist, self._cmd_topic, 10)
        self._pub_complete = self.create_publisher(Bool, self._complete_topic, qos)

        self._phase = self._PH_INIT_WAIT
        self._phase_start = self.get_clock().now()
        self._published_done = False

        rate = float(self.get_parameter("timer_rate_hz").get_parameter_value().double_value)
        if rate <= 0.0:
            rate = 50.0
        period = 1.0 / rate
        self._timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            "startup_map_warmup_node: delay=%.1fs, ±%.0f° @ %.2f rad/s (~%.2fs each leg), cmd_vel=%s"
            % (
                self._startup_delay,
                math.degrees(angle_rad),
                self._omega,
                self._turn_duration,
                self._cmd_topic,
            )
        )

    def _phase_elapsed(self) -> float:
        return (self.get_clock().now() - self._phase_start).nanoseconds / 1e9

    def _next_phase(self, new_phase: int) -> None:
        self._phase = new_phase
        self._phase_start = self.get_clock().now()

    def _send_turn(self, sign: float) -> None:
        t = Twist()
        t.angular.z = sign * self._omega
        self._pub_cmd.publish(t)

    def _send_stop(self) -> None:
        self._pub_cmd.publish(Twist())

    def _tick(self) -> None:
        if self._phase == self._PH_DONE:
            return

        dt = self._phase_elapsed()

        if self._phase == self._PH_INIT_WAIT:
            self._send_stop()
            if dt >= self._startup_delay:
                self.get_logger().info("Warmup: turn left ~45°")
                self._next_phase(self._PH_TURN_LEFT)
            return

        if self._phase == self._PH_TURN_LEFT:
            self._send_turn(1.0)
            if dt >= self._turn_duration:
                self._send_stop()
                self._next_phase(self._PH_PAUSE_1)
            return

        if self._phase == self._PH_PAUSE_1:
            self._send_stop()
            if dt >= self._pause:
                self.get_logger().info("Warmup: return to forward")
                self._next_phase(self._PH_RETURN_1)
            return

        if self._phase == self._PH_RETURN_1:
            self._send_turn(-1.0)
            if dt >= self._turn_duration:
                self._send_stop()
                self._next_phase(self._PH_PAUSE_2)
            return

        if self._phase == self._PH_PAUSE_2:
            self._send_stop()
            if dt >= self._pause:
                self.get_logger().info("Warmup: turn right ~45°")
                self._next_phase(self._PH_TURN_RIGHT)
            return

        if self._phase == self._PH_TURN_RIGHT:
            self._send_turn(-1.0)
            if dt >= self._turn_duration:
                self._send_stop()
                self._next_phase(self._PH_PAUSE_3)
            return

        if self._phase == self._PH_PAUSE_3:
            self._send_stop()
            if dt >= self._pause:
                self.get_logger().info("Warmup: return to forward (complete)")
                self._next_phase(self._PH_RETURN_2)
            return

        if self._phase == self._PH_RETURN_2:
            self._send_turn(1.0)
            if dt >= self._turn_duration:
                self._send_stop()
                self._finish()

    def _finish(self) -> None:
        self._phase = self._PH_DONE
        self._timer.cancel()
        self._pub_complete.publish(Bool(data=True))
        self._published_done = True
        self.get_logger().info(
            "Published exploration warmup complete on '%s' (latched)." % self._complete_topic
        )


def main(args=None):
    rclpy.init(args=args)
    node = StartupMapWarmupNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    if not node._published_done:
        node._pub_complete.publish(Bool(data=True))
        node.get_logger().warn("Interrupted; published warmup complete so exploration is not blocked.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
