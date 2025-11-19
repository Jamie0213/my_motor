#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import sys
import time
import threading
import argparse

from rclpy.utilities import remove_ros_args
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from evdev import InputDevice, ecodes

# -------------------------------------------------------------
# 조이스틱 입력 리더
# -------------------------------------------------------------
class JoystickReader:
    def __init__(self, device_path="/dev/input/event4"):
        self.dev = InputDevice(device_path)

        self.steer_raw = 0       # ABS_X
        self.throttle_raw = 0    # ABS_RY

        self.DEADZONE = 3000

        print(f"[JOYSTICK] Opened {device_path}")

    def normalize(self, v):
        if abs(v) < self.DEADZONE:
            return 0.0
        return float(v) / 32767.0

    def start(self):
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        for event in self.dev.read_loop():
            if event.type == ecodes.EV_ABS:

                # 왼쪽 스틱 X = steering
                if event.code == ecodes.ABS_X:
                    self.steer_raw = self.normalize(event.value)

                # 오른쪽 스틱 Y = throttle
                elif event.code == ecodes.ABS_RY:
                    # 위(-32767) → 전진(+)
                    self.throttle_raw = -self.normalize(event.value)


# ============================================================
# 기존 UDP 수신부 기반 → 조이스틱 입력으로 수정
# ============================================================
class JoystickAckermann(Node):
    def __init__(self, args: argparse.Namespace, joystick: JoystickReader):
        super().__init__("joystick_ackermann")
        self.args = args
        self.joystick = joystick

        self.pub = self.create_publisher(Float32MultiArray, args.topic, 1)

        # 30Hz publish
        self.timer = self.create_timer(1.0 / 30.0, self.publish_latest)

        self.get_logger().info(
            f"[JOYSTICK] Publishing to '{args.topic}' "
            f"(angle_scale={args.angle_scale}, speed_scale={args.speed_scale})"
        )

    def publish_latest(self):
        # 조이스틱 값 (-1.0 ~ +1.0)
        steer = self.joystick.steer_raw
        throttle = self.joystick.throttle_raw

        # int32 포맷처럼 스케일 변환
        angle_i = int(steer * 3000)
        speed_i = int(throttle * 1000)

        # 기존 convert 함수 적용
        angle_out, speed_out, _ = self._convert(angle_i, speed_i)

        msg = Float32MultiArray()
        msg.data = [float(angle_out), float(speed_out)]
        self.pub.publish(msg)

    def _convert(self, angle_i: int, speed_i: int):
        steer_rad = float(angle_i) / max(1e-6, self.args.angle_scale)
        if self.args.angle_invert:
            steer_rad = -steer_rad

        angle_out = steer_rad if self.args.angle_unit == "rad" else math.degrees(steer_rad)
        speed_out = float(speed_i) * self.args.speed_scale

        # limit 적용
        if self.args.angle_limit is not None:
            angle_out = max(-abs(self.args.angle_limit), min(abs(self.args.angle_limit), angle_out))
        if self.args.speed_limit is not None:
            speed_out = max(-abs(self.args.speed_limit), min(abs(self.args.speed_limit), speed_out))

        return angle_out, speed_out, steer_rad


# ============================================================
# argparse (기존 UDP 코드 거의 그대로 유지)
# ============================================================
def build_argparser():
    p = argparse.ArgumentParser(description="ROS2 Joystick Ackermann publisher")
    p.add_argument("--joystick", default="/dev/input/event5")
    p.add_argument("--angle-scale", type=float, default=50.0)
    p.add_argument("--angle-invert", action="store_true")
    p.add_argument("--speed-scale", type=float, default=0.08)
    p.add_argument("--angle-unit", choices=["deg", "rad"], default="rad")
    p.add_argument("--angle-limit", type=float, default=50.0)
    p.add_argument("--speed-limit", type=float, default=8.0)
    p.add_argument("--topic", default="/JOYSTICK_CMD")
    return p


def main():
    filtered = remove_ros_args(sys.argv)
    args = build_argparser().parse_args(filtered[1:])

    # 조이스틱 준비
    joystick = JoystickReader(args.joystick)
    joystick.start()

    # ROS 시작
    rclpy.init(args=None)
    node = JoystickAckermann(args, joystick)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
