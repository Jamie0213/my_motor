#!/usr/bin/env python3

from typing import List, Dict

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Int32, Int32MultiArray


class UltrasonicRepublisher(Node):
    """`xycar_ultrasonic` 배열 토픽을 센서별 cm 정수 토픽으로 분리."""

    SENSOR_ORDER: List[Dict[str, object]] = [
        {"index": 3, "name": "left"},
        {"index": 4, "name": "right"},
        {"index": 5, "name": "back_right"},
        {"index": 6, "name": "back_center"},
        {"index": 7, "name": "back_left"},
    ]

    def __init__(self) -> None:
        super().__init__("ultrasonic_republisher")

        # 파라미터 (필요 시 launch에서 override 가능)
        self.declare_parameter("input_topic", "xycar_ultrasonic")
        self.declare_parameter("min_distance_cm", 0.0)
        self.declare_parameter("max_distance_cm", 140.0)

        input_topic = (
            self.get_parameter("input_topic").get_parameter_value().string_value
        )

        min_cm = float(self.get_parameter("min_distance_cm").value)
        max_cm = float(self.get_parameter("max_distance_cm").value)
        if min_cm < 0 or min_cm >= max_cm:
            self.get_logger().warn(
                "Invalid min/max distance parameters, reverting to defaults."
            )
            min_cm = 2.0
            max_cm = 140.0

        self.min_range_cm = min_cm
        self.max_range_cm = max_cm

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.sensor_publishers = {
            sensor["name"]: self.create_publisher(
                Int32,
                f"/ultrasonic/{sensor['name']}",
                qos,
            )
            for sensor in self.SENSOR_ORDER
        }

        self.subscription = self.create_subscription(
            Int32MultiArray, input_topic, self._callback, qos
        )

        self.get_logger().info(
            "Ultrasonic republisher ready: "
            + ", ".join(f"/ultrasonic/{s['name']}" for s in self.SENSOR_ORDER)
        )

    def _callback(self, msg: Int32MultiArray) -> None:
        data = list(msg.data)
        if len(data) < max(sensor["index"] for sensor in self.SENSOR_ORDER) + 1:
            self.get_logger().warn(
                f"Received insufficient data length: {len(data)}",
                throttle_duration_sec=5.0,
            )
            return

        for sensor in self.SENSOR_ORDER:
            index = sensor["index"]
            raw_cm = data[index]
            topic_name = sensor["name"]

            cm_msg = Int32()
            if raw_cm <= self.min_range_cm or raw_cm > self.max_range_cm:
                cm_msg.data = 0
            else:
                cm_msg.data = int(raw_cm)
            self.sensor_publishers[topic_name].publish(cm_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UltrasonicRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

