#!/usr/bin/env python3

import socket
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Int32MultiArray


class UltrasonicUdpSender(Node):
    """xycar_ultrasonic 토픽을 받아 UDP로 다른 장치에 전송."""

    SENSOR_INDICES: List[int] = [3, 4, 5, 6, 7]  # left, right, back_right, back_center, back_left

    def __init__(self) -> None:
        super().__init__("ultrasonic_udp_sender")

        self.declare_parameter("input_topic", "xycar_ultrasonic")
        self.declare_parameter("remote_ip", "10.15.129.52)")
        self.declare_parameter("remote_port", 6000)
        self.declare_parameter("send_hz", 30.0)

        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        self.remote_ip = self.get_parameter("remote_ip").get_parameter_value().string_value
        self.remote_port = int(self.get_parameter("remote_port").value)
        send_hz = float(self.get_parameter("send_hz").value)
        if send_hz <= 0:
            self.get_logger().warn("send_hz must be positive. defaulting to 30Hz.")
            send_hz = 30.0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.subscription = self.create_subscription(
            Int32MultiArray, input_topic, self._callback, qos
        )

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)

        self.latest_values: List[int] = [0] * len(self.SENSOR_INDICES)
        self.last_seq = 0

        timer_period = 1.0 / send_hz
        self.timer = self.create_timer(timer_period, self._send_packet)

        self.get_logger().info(
            f"Ultrasonic UDP sender ready -> {self.remote_ip}:{self.remote_port} "
            f"(topic={input_topic}, rate={send_hz}Hz)"
        )

    def _callback(self, msg: Int32MultiArray) -> None:
        data = list(msg.data)
        if max(self.SENSOR_INDICES) >= len(data):
            self.get_logger().warn(
                f"Received insufficient ultrasonic data length: {len(data)}",
                throttle_duration_sec=5.0,
            )
            return

        self.latest_values = [max(0, int(data[i])) for i in self.SENSOR_INDICES]
        self.last_seq += 1

    def _send_packet(self) -> None:
        if self.last_seq == 0:
            return  # 아직 데이터 없음

        payload = ",".join(str(v) for v in self.latest_values).encode("utf-8")
        try:
            self.sock.sendto(payload, (self.remote_ip, self.remote_port))
        except OSError as exc:
            self.get_logger().warn(f"UDP send failed: {exc}", throttle_duration_sec=5.0)

    def destroy_node(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UltrasonicUdpSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

