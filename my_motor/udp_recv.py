#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDrive           # input
from std_msgs.msg import Float32MultiArray              # output


class AckermannToXycar(Node):
    def __init__(self):
        super().__init__('ackermann_to_xycar')

        # ---- Parameters (with sane defaults) ----
        self.declare_parameter('ackermann_topic', '/carla/ego_vehicle_1/vehicle_control_cmd')
        self.declare_parameter('xycar_topic', 'xycar_motor')

        self.declare_parameter('angle_scale', 500.0)   # steer(rad) * scale -> servo range
        self.declare_parameter('angle_clip', 50)       # final clamp for angle
        self.declare_parameter('angle_min_abs', 8)     # deadzone fix (min abs angle)
        self.declare_parameter('angle_invert', False)  # invert servo direction

        self.declare_parameter('speed_clip', 50)       # clamp speed to [-50, 50]

        ackermann_topic = self.get_parameter('ackermann_topic').get_parameter_value().string_value
        xycar_topic      = self.get_parameter('xycar_topic').get_parameter_value().string_value

        self.scale   = float(self.get_parameter('angle_scale').value)
        self.clip    = int(self.get_parameter('angle_clip').value)
        self.min_abs = int(self.get_parameter('angle_min_abs').value)
        self.invert  = bool(self.get_parameter('angle_invert').value)
        self.sp_clip = int(self.get_parameter('speed_clip').value)

        # ---- Pub/Sub ----
        self.pub = self.create_publisher(Float32MultiArray, xycar_topic, 10)
        self.sub = self.create_subscription(AckermannDrive, ackermann_topic, self._cb, 10)

        self.get_logger().info(
            f'Bridge ready: {ackermann_topic} (AckermannDrive) -> {xycar_topic} (Float32MultiArray [angle, speed])'
        )

    def _cb(self, msg: AckermannDrive):
        # Convert steering angle in radians to servo angle
        steer = float(msg.steering_angle)          # rad
        speed = float(msg.speed)

        xy_angle = int(round(steer * self.scale))
        if 0 < abs(xy_angle) < self.min_abs:
            xy_angle = self.min_abs if xy_angle > 0 else -self.min_abs
        if self.invert:
            xy_angle = -xy_angle
        xy_angle = max(-self.clip, min(self.clip, xy_angle))

        xy_speed = int(round(speed))
        xy_speed = max(-self.sp_clip, min(self.sp_clip, xy_speed))

        out = Float32MultiArray()
        out.data = [float(xy_angle), float(xy_speed)]
        self.pub.publish(out)

        # light-rate log
        self.get_logger().debug(
            f'[xycar_motor] angle={xy_angle:>3}, speed={xy_speed:>3}, '
            f'steer={steer:.3f}rad ({math.degrees(steer):.1f}°)'
        )


def main(args=None):
    rclpy.init(args=args)
    node = AckermannToXycar()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
