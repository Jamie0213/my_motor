#!/usr/bin/env python3
# go.py — ROS2 publisher for RC car from UDP Ackermann packets (INT/FLOAT with selectable output mapping)

import argparse
import math
import socket
import struct
import sys
import time
from typing import Tuple, Optional

from rclpy.utilities import remove_ros_args
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

# Wire formats (network byte order)
FMT_INT = "!iiI"    # int32 angle_i, int32 speed_i, uint32 seq
FMT_FLOAT = "!fiI"  # float angle_rad, int32 speed_i, uint32 seq
PKT_SIZE_INT = struct.calcsize(FMT_INT)
PKT_SIZE_FLOAT = struct.calcsize(FMT_FLOAT)


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class UdpAckermannReceiver(Node):
    """
    fmt:
      - auto   : payload len based auto detect (float first)
      - int    : force INT
      - float  : force FLOAT

    angle-out:
      - rad    : publish radians
      - deg    : publish degrees
      - xycar  : publish "vehicle units" (default 1 unit = 1 deg, clamped to ±50)
    """

    def __init__(self, args: argparse.Namespace):
        super().__init__("udp_ackermann_receiver")
        self.args = args

        # ROS Publisher
        self.pub = self.create_publisher(Float32MultiArray, args.topic, 1)

        # UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 512 * 1024)
        except OSError:
            pass
        self.sock.bind((args.bind_ip, args.bind_port))
        self.sock.setblocking(False)

        # State
        self.last_seq: int = -1
        self.latest: Optional[Tuple[str, float, int, int, tuple]] = None
        # latest = (fmt, steer_rad, speed_i, seq, addr)
        self.last_recv_time: float = 0.0
        self.last_print: float = 0.0

        # Timers
        self.timer = self.create_timer(args.poll_sec, self.poll_once)
        self.pub_timer = self.create_timer(1.0 / max(1.0, float(args.pub_hz)), self.publish_latest)

        # Derived defaults for limits
        if self.args.angle_out == "xycar":
            # XYCar는 보통 ±50 단위
            if self.args.angle_limit is None:
                self.args.angle_limit = 50.0
        elif self.args.angle_out == "deg":
            if self.args.angle_limit is None:
                self.args.angle_limit = 50.0
        # rad 모드는 기본 제한 없음

        self.get_logger().info(
            "[RC-UDP-RECV] listen %s:%d → topic='%s' fmt=%s | angle_out=%s "
            "angle_scale(INT)=%.6g invert=%s angle_limit=%s xycar_unit_per_deg=%.6g "
            "speed_scale=%.6g speed_limit=%s pub_hz=%.1f"
            % (
                args.bind_ip, args.bind_port, args.topic, args.fmt, args.angle_out,
                args.angle_scale, args.angle_invert,
                ("none" if args.angle_limit is None else str(args.angle_limit)),
                args.xycar_unit_per_deg, args.speed_scale,
                ("none" if args.speed_limit is None else str(args.speed_limit)),
                args.pub_hz,
            )
        )

    # --------------- decode ---------------

    def _decode_packet(self, data: bytes) -> Optional[Tuple[str, float, int, int]]:
        """
        Returns (fmt, steer_rad, speed_i, seq) or None
        """
        mode = self.args.fmt

        if mode == "auto":
            # prefer float if possible
            if len(data) >= PKT_SIZE_FLOAT:
                try:
                    angle_f, speed_i, seq = struct.unpack(FMT_FLOAT, data[:PKT_SIZE_FLOAT])
                    return ("float", float(angle_f), int(speed_i), int(seq))
                except struct.error:
                    pass
            if len(data) >= PKT_SIZE_INT:
                try:
                    angle_i, speed_i, seq = struct.unpack(FMT_INT, data[:PKT_SIZE_INT])
                    steer_rad = float(angle_i) / max(1e-6, self.args.angle_scale)
                    return ("int", float(steer_rad), int(speed_i), int(seq))
                except struct.error:
                    pass
            return None

        if mode == "float":
            if len(data) < PKT_SIZE_FLOAT:
                return None
            angle_f, speed_i, seq = struct.unpack(FMT_FLOAT, data[:PKT_SIZE_FLOAT])
            return ("float", float(angle_f), int(speed_i), int(seq))

        # mode == "int"
        if len(data) < PKT_SIZE_INT:
            return None
        angle_i, speed_i, seq = struct.unpack(FMT_INT, data[:PKT_SIZE_INT])
        steer_rad = float(angle_i) / max(1e-6, self.args.angle_scale)
        return ("int", float(steer_rad), int(speed_i), int(seq))

    # --------------- mapping ---------------

    def _map_angle_for_publish(self, steer_rad: float) -> float:
        # invert sign after decoding if requested
        if self.args.angle_invert:
            steer_rad = -steer_rad

        if self.args.angle_out == "rad":
            angle_out = steer_rad
        elif self.args.angle_out == "deg":
            angle_out = math.degrees(steer_rad)
        else:  # xycar
            angle_deg = math.degrees(steer_rad)
            angle_out = angle_deg * self.args.xycar_unit_per_deg

        # clamp
        if self.args.angle_limit is not None:
            angle_out = clamp(angle_out, -abs(self.args.angle_limit), abs(self.args.angle_limit))
        return float(angle_out)

    def _map_speed_for_publish(self, speed_i: int) -> float:
        speed_out = float(speed_i) * self.args.speed_scale
        if self.args.speed_limit is not None:
            speed_out = clamp(speed_out, -abs(self.args.speed_limit), abs(self.args.speed_limit))
        return float(speed_out)

    # --------------- timers ---------------

    def poll_once(self):
        try:
            while True:
                data, addr = self.sock.recvfrom(64)
                decoded = self._decode_packet(data)
                if decoded is None:
                    continue

                fmt, steer_rad, speed_i, seq = decoded

                # seq monotonicity with reset tolerance
                if self.last_seq != -1 and seq <= self.last_seq:
                    if self.last_seq > 100000 and seq < 100:
                        self.get_logger().warn("[RC-UDP] Sequence reset detected (new session).")
                        self.last_seq = -1
                    else:
                        continue

                self.last_seq = seq
                self.latest = (fmt, steer_rad, speed_i, seq, addr)
                self.last_recv_time = time.time()

        except BlockingIOError:
            pass
        except Exception as e:
            self.get_logger().warn(f"[RC-UDP] recvfrom error: {e} → reinit socket...")
            try:
                self.sock.close()
            except Exception:
                pass
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 512 * 1024)
                self.sock.bind((self.args.bind_ip, self.args.bind_port))
                self.sock.setblocking(False)
                self.get_logger().info("[RC-UDP] Socket reinitialized.")
            except Exception as e2:
                self.get_logger().error(f"[RC-UDP] Failed to rebind socket: {e2}")

    def publish_latest(self):
        if self.latest is None:
            return
        if (time.time() - self.last_recv_time) > self.args.timeout_sec:
            msg = Float32MultiArray()
            msg.data = [0.0, 0.0]
            self.pub.publish(msg)
            self.get_logger().warn("[RC-UDP] Timeout → stopping motors. (no packets)")
            self.latest = None
            self.last_seq = -1
            return

        fmt, steer_rad, speed_i, seq, addr = self.latest
        angle_out = self._map_angle_for_publish(steer_rad)
        speed_out = self._map_speed_for_publish(speed_i)

        msg = Float32MultiArray()
        msg.data = [angle_out, speed_out]
        self.pub.publish(msg)

        now = time.time()
        if now - self.last_print >= self.args.throttle_sec:
            if fmt == "int":
                raw_i = int(round(steer_rad * self.args.angle_scale))
                self.get_logger().info(
                    f"[INT] from={addr[0]}:{addr[1]} seq={seq} pure_angle_i={raw_i} "
                    f"steer={angle_out:.4f} {self.args.angle_out} speed={speed_out:.2f}"
                )
            else:
                self.get_logger().info(
                    f"[FLOAT] from={addr[0]}:{addr[1]} seq={seq} pure_angle_f(rad)={steer_rad:.4f} "
                    f"steer={angle_out:.4f} {self.args.angle_out} speed={speed_out:.2f}"
                )
            self.last_print = now

    def destroy_node(self):
        try:
            self.sock.close()
        except Exception:
            pass
        super().destroy_node()


def build_argparser():
    p = argparse.ArgumentParser(description="ROS2 UDP Ackermann receiver -> Float32MultiArray publisher")
    # network
    p.add_argument("--bind-ip", default="0.0.0.0")
    p.add_argument("--bind-port", type=int, default=5555)
    # format
    p.add_argument("--fmt", choices=["auto", "int", "float"], default="auto")
    # steering conversion (INT only)
    p.add_argument("--angle-scale", type=float, default=50.0, help="INT: steer_rad = angle_i / angle_scale")
    p.add_argument("--angle-invert", action="store_true", help="Invert steering after decoding")
    # output mapping
    p.add_argument("--angle-out", choices=["rad", "deg", "xycar"], default="xycar",
                   help="Publish unit for angle")
    p.add_argument("--xycar-unit-per-deg", type=float, default=1.0,
                   help="XYCar mode: unit = deg * unit_per_deg (default 1 unit = 1 deg)")
    p.add_argument("--angle-limit", type=float, default=None, help="Clamp published angle (unit depends on angle-out)")
    # speed mapping
    p.add_argument("--speed-scale", type=float, default=1.0, help="speed = speed_i * scale")
    p.add_argument("--speed-limit", type=float, default=100.0, help="Clamp published speed")
    # runtime
    p.add_argument("--topic", default="/UDP_RECV_CMD")
    # p.add_argument("--topic", default="/xycar_motor")
    p.add_argument("--pub-hz", type=float, default=30.0)
    p.add_argument("--throttle-sec", type=float, default=0.2)
    p.add_argument("--poll-sec", type=float, default=0.01)
    p.add_argument("--timeout-sec", type=float, default=0.5)
    return p


def normalize_args(args: argparse.Namespace) -> argparse.Namespace:
    # negative limits disable
    if args.angle_limit is not None and args.angle_limit < 0:
        args.angle_limit = None
    if args.speed_limit is not None and args.speed_limit < 0:
        args.speed_limit = None
    return args


def main(argv=None) -> int:
    filtered = remove_ros_args(sys.argv)
    args = build_argparser().parse_args(filtered[1:])
    args = normalize_args(args)

    rclpy.init(args=None)
    node = UdpAckermannReceiver(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[RC-UDP-RECV] interrupted, exiting", file=sys.stderr)
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            # guard against "already shutdown"
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
