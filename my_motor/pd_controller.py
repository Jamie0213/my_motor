#!/usr/bin/env python3

from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Int32
from std_msgs.msg import Float32MultiArray


@dataclass
class UltrasonicState:
    back_left: Optional[int] = None
    back_center: Optional[int] = None
    back_right: Optional[int] = None


class PdController(Node):
    """Maintain rear distance around target using PD control while driving straight."""

    def __init__(self) -> None:
        super().__init__("pd_controller")

        # Parameters
        self.target_distance_cm = float(self.declare_parameter("target_distance_cm", 60.0).value)
        self.tolerance_cm = float(self.declare_parameter("tolerance_cm", 5.0).value)
        self.base_speed = float(self.declare_parameter("base_speed", 6.0).value)
        self.min_speed = float(self.declare_parameter("min_speed", 3.0).value)
        self.max_speed = float(self.declare_parameter("max_speed", 12.0).value)

        self.kp_speed = float(self.declare_parameter("kp_speed", 0.075).value)
        self.kd_speed = float(self.declare_parameter("kd_speed", 0.05).value)
        self.max_speed_delta = float(self.declare_parameter("max_speed_delta", 22.5).value)
        self.speed_filter_alpha = float(self.declare_parameter("speed_filter_alpha", 0.7).value)

        self.kp_steer = float(self.declare_parameter("kp_steer", 0.02).value)
        self.steering_offset = float(self.declare_parameter("steering_offset", 0.0).value)
        self.max_steer_angle = float(self.declare_parameter("max_steer_angle", 30.0).value)

        self.command_topic = self.declare_parameter("command_topic", "xycar_motor").get_parameter_value().string_value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Ultrasonic subscriptions
        self.state = UltrasonicState()
        self.create_subscription(Int32, "/ultrasonic/back_left", self._on_back_left, qos)
        self.create_subscription(Int32, "/ultrasonic/back_center", self._on_back_center, qos)
        self.create_subscription(Int32, "/ultrasonic/back_right", self._on_back_right, qos)

        # Publisher to motor topic
        self.motor_pub = self.create_publisher(Float32MultiArray, self.command_topic, 10)

        # Control timer
        control_hz = float(self.declare_parameter("control_hz", 30.0).value)
        self.control_period = 1.0 / max(control_hz, 1.0)
        self.timer = self.create_timer(self.control_period, self._control_step)

        # PD states
        self.prev_error = 0.0
        self.prev_time = self.get_clock().now()
        self.prev_speed_command = self.base_speed

        self.get_logger().info(
            f"PD controller ready (target={self.target_distance_cm}cm ±{self.tolerance_cm}cm, "
            f"base_speed={self.base_speed}, kp={self.kp_speed}, kd={self.kd_speed})"
        )

    def _on_back_left(self, msg: Int32) -> None:
        self.state.back_left = msg.data

    def _on_back_center(self, msg: Int32) -> None:
        self.state.back_center = msg.data

    def _on_back_right(self, msg: Int32) -> None:
        self.state.back_right = msg.data

    def _control_step(self) -> None:
        if not self._has_full_data():
            self.get_logger().debug("Waiting for ultrasonic data...")
            return

        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds / 1e9
        if dt <= 0:
            dt = self.control_period

        distance = float(self.state.back_center or 0)
        error = self.target_distance_cm - distance

        # PD speed control
        derivative = (error - self.prev_error) / dt
        control_speed = self.base_speed + (self.kp_speed * error) + (self.kd_speed * derivative)

        # Within tolerance -> hold base speed
        if abs(error) <= self.tolerance_cm:
            control_speed = self.base_speed

        control_speed = max(self.min_speed, min(self.max_speed, control_speed))

        # Limit rate of change
        delta = control_speed - self.prev_speed_command
        max_delta = self.max_speed_delta
        if delta > max_delta:
            control_speed = self.prev_speed_command + max_delta
        elif delta < -max_delta:
            control_speed = self.prev_speed_command - max_delta

        # Low-pass filter
        alpha = max(0.0, min(1.0, self.speed_filter_alpha))
        control_speed = (alpha * control_speed) + ((1.0 - alpha) * self.prev_speed_command)

        # Steering control based on difference between back sensors
        diff = float((self.state.back_right or 0) - (self.state.back_left or 0))
        steer_angle = self.kp_steer * diff
        steer_angle += self.steering_offset
        steer_angle = max(-self.max_steer_angle, min(self.max_steer_angle, steer_angle))

        msg = Float32MultiArray()
        msg.data = [float(steer_angle), float(control_speed)]
        self.motor_pub.publish(msg)

        self.prev_error = error
        self.prev_speed_command = control_speed
        self.prev_time = now

        self.get_logger().debug(
            f"dist={distance:.1f}cm err={error:.1f} ctl_speed={control_speed:.2f} steer={steer_angle:.2f}"
        )

    def _has_full_data(self) -> bool:
        return (
            self.state.back_left is not None
            and self.state.back_center is not None
            and self.state.back_right is not None
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PdController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

