#!/usr/bin/env python3

import argparse
import sys
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Float32MultiArray


def _copy_array(msg: Float32MultiArray) -> Float32MultiArray:
    copied = Float32MultiArray()
    copied.data = list(msg.data)
    return copied

ESTOP = 2
MANUAL_MODE = 1
AD_MODE = 0

class AckermannCommandMux(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("xycar_command_mux")
        self.args = args

        self.publisher = self.create_publisher(Float32MultiArray, args.output_topic, 10)
        self.udp_sub = self.create_subscription(
            Float32MultiArray, args.udp_topic, self._on_udp, 10
        )
        self.joystick_sub = self.create_subscription(
            Float32MultiArray, args.joystick_topic, self._on_joystick, 10
        )

        self.mode = MANUAL_MODE

        period = 1.0 / max(1.0, float(args.pub_hz))
        self.timer = self.create_timer(period, self._on_timer)

        self.zero_msg = Float32MultiArray()
        self.zero_msg.data = [0.0, 0.0]

        self.last_udp_msg: Optional[Float32MultiArray] = None
        self.last_udp_time: Optional[float] = None

        self.last_joy_msg: Optional[Float32MultiArray] = None
        self.last_joy_time: Optional[float] = None

        self.last_source: Optional[str] = None

        self.joystick_deadband = abs(args.joystick_deadband)

        self.get_logger().info(
            "[MUX] Listening udp='%s' joystick='%s' → output='%s' (joy_timeout=%.3fs, udp_timeout=%.3fs, pub_hz=%.1f)"
            % (
                args.udp_topic,
                args.joystick_topic,
                args.output_topic,
                args.joystick_timeout,
                args.udp_timeout,
                args.pub_hz,
            )
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_udp(self, msg: Float32MultiArray):
        self.last_udp_msg = _copy_array(msg)
        self.last_udp_time = self._now()

    def _on_joystick(self, msg: Float32MultiArray):
        self.last_joy_msg = _copy_array(msg)
        self.last_joy_time = self._now()

    def _is_joystick_active(self) -> bool:
        if self.last_joy_msg is None:
            return False
        if len(self.last_joy_msg.data) == 0:
            return False
        for value in self.last_joy_msg.data[:2]:
            if abs(value) > self.joystick_deadband or self.mode == MANUAL_MODE:
                return True
        return False

    def _select_command(self, now: float) -> Tuple[str, Float32MultiArray]:
        joy_fresh = (
            self.last_joy_msg is not None
            and self.last_joy_time is not None
            and (now - self.last_joy_time) <= self.args.joystick_timeout
        )
        udp_fresh = (
            self.last_udp_msg is not None
            and self.last_udp_time is not None
            and (now - self.last_udp_time) <= self.args.udp_timeout
        )

        self.get_logger().info(f'joy : {joy_fresh} udp : {udp_fresh}')

        if joy_fresh and self._is_joystick_active():
            print("joystick_fresh")
            return "joystick", _copy_array(self.last_joy_msg)

        if udp_fresh:
            print("udp_fresh")
            return "udp", _copy_array(self.last_udp_msg)
        return "idle", self.zero_msg

    def _on_timer(self):
        now = self._now()

        joy_fresh = (
            self.last_joy_msg is not None
            and self.last_joy_time is not None
            and (now - self.last_joy_time) <= self.args.joystick_timeout
        )
        
        udp_fresh = (
            self.last_udp_msg is not None
            and self.last_udp_time is not None
            and (now - self.last_udp_time) <= self.args.udp_timeout
        )

        self.get_logger().info(f'joy : {joy_fresh} udp : {udp_fresh}')

        pub_msg = Float32MultiArray()

        if joy_fresh:
            self.mode = int(self.last_joy_msg.data[2])

        if udp_fresh and self.mode == AD_MODE:
            pub_msg.data = self.last_udp_msg.data[:2]
            self.publisher.publish(pub_msg)

        elif joy_fresh and self.mode == MANUAL_MODE:
            pub_msg.data = self.last_joy_msg.data[:2]
            self.publisher.publish(pub_msg)

        else:
            pub_msg.data = [0., 0.]
            self.publisher.publish(pub_msg)

        mode = ["AD_MODE", "MANUAL_MODE", "ESTOP"][self.mode]

        self.get_logger().info(f"{mode}")

        # if source != self.last_source:
        #     if source == "joystick":
        #         self.get_logger().info("[MUX] Using joystick command (priority).")
        #     elif source == "udp":
        #         self.get_logger().info("[MUX] Using UDP command.")
        #     else:
        #         self.get_logger().warn("[MUX] No fresh command → stop (0,0).")
        #     self.last_source = source


def build_argparser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="ROS2 Ackermann command multiplexer for XYcar motor control."
    )
    p.add_argument("--udp-topic", default="/UDP_RECV_CMD")
    p.add_argument("--joystick-topic", default="/JOYSTICK_CMD")
    p.add_argument("--output-topic", default="/xycar_motor")
    p.add_argument("--pub-hz", type=float, default=30.0)
    p.add_argument("--udp-timeout", type=float, default=0.5)
    p.add_argument("--joystick-timeout", type=float, default=0.3)
    p.add_argument("--joystick-deadband", type=float, default=0.05)
    return p


def main(argv=None) -> int:
    filtered = remove_ros_args(sys.argv)
    args = build_argparser().parse_args(filtered[1:])

    rclpy.init(args=None)
    node = AckermannCommandMux(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

