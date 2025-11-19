#!/usr/bin/env python3
# udp_recv.py — ROS2 publisher for RC car from UDP Ackermann packets
import argparse
import math
import socket
import struct
import sys
import time
from typing import Tuple
from rclpy.utilities import remove_ros_args

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

FMT = "!iiI"  # int32 angle_i, int32 speed_i, uint32 seq


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class UdpAckermannReceiver(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("udp_ackermann_receiver")
        self.args = args

        # ROS Publisher
        self.pub = self.create_publisher(Float32MultiArray, args.topic, 1)

        # UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 512 * 1024)
        self.sock.bind((args.bind_ip, args.bind_port))
        self.sock.setblocking(False)

        self.pkt_size = struct.calcsize(FMT)
        self.last_seq = -1
        self.latest_data = None
        self.last_recv_time = 0.0
        self.last_print = 0.0

        # Timers
        self.timer = self.create_timer(args.poll_sec, self.poll_once)
        self.pub_timer = self.create_timer(1.0 / 30.0, self.publish_latest)

        self.get_logger().info(
            f"[RC-UDP-RECV] listening on {args.bind_ip}:{args.bind_port} -> publishing to '{args.topic}' "
            f"(angle_unit={args.angle_unit}, angle_limit={args.angle_limit}, speed_limit={args.speed_limit})"
        )

    def _convert(self, angle_i: int, speed_i: int) -> Tuple[float, float, float]:
        # steering rad 계산 (scale 적용)
        steer_rad = float(angle_i) / max(1e-6, self.args.angle_scale)
        if self.args.angle_invert:
            steer_rad = -steer_rad

        # 출력 단위 선택
        if self.args.angle_unit == "deg":
            angle_out = math.degrees(steer_rad)
        else:
            angle_out = steer_rad

        # 속도 변환
        speed_out = float(speed_i) * self.args.speed_scale

        # 제한
        if self.args.angle_limit is not None:
            angle_out = clamp(angle_out, -abs(self.args.angle_limit), abs(self.args.angle_limit))
        if self.args.speed_limit is not None:
            speed_out = clamp(speed_out, -abs(self.args.speed_limit), abs(self.args.speed_limit))

        return angle_out, speed_out, steer_rad

    # ============================================================
    # 🔧 개선된 poll_once(): 재연결 및 시퀀스 리셋 지원
    # ============================================================
    def poll_once(self):
        try:
            while True:
                data, addr = self.sock.recvfrom(64)
                if len(data) < self.pkt_size:
                    continue

                angle_i, speed_i, seq = struct.unpack(FMT, data[:self.pkt_size])

                # ✅ seq 비교 시 last_seq == -1이면 새 세션으로 간주
                if self.last_seq != -1 and seq <= self.last_seq:
                    continue

                # ✅ 시퀀스가 새로 초기화된 것으로 감지
                if self.last_seq != -1 and seq < 100 and self.last_seq > 100000:
                    self.get_logger().warn("[RC-UDP] Sequence reset detected (new session).")

                self.last_seq = seq
                self.latest_data = (angle_i, speed_i, seq, addr)
                self.last_recv_time = time.time()

        except BlockingIOError:
            pass
        except Exception as e:
            # ⚠️ 에러 발생 시 소켓 재생성
            self.get_logger().warn(f"[RC-UDP] recvfrom error: {e}, reconnecting socket...")
            try:
                self.sock.close()
            except Exception:
                pass

            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 512 * 1024)
                self.sock.bind((self.args.bind_ip, self.args.bind_port))
                self.sock.setblocking(False)
                self.get_logger().info("[RC-UDP] Socket reinitialized successfully.")
            except Exception as e2:
                self.get_logger().error(f"[RC-UDP] Failed to rebind socket: {e2}")

    def publish_latest(self):
        if self.latest_data is None:
            return

        angle_i, speed_i, seq, addr = self.latest_data

        # ✅ 0.5초 이상 새 데이터 없으면 안전 정지 + seq 초기화
        if time.time() - self.last_recv_time > 0.5:
            msg = Float32MultiArray()
            msg.data = [0.0, 0.0]
            self.pub.publish(msg)
            self.get_logger().warn("[RC-UDP] Timeout → stopping motors.")
            self.latest_data = None
            self.last_seq = -1  # ✅ 새 세션 수용을 위해 시퀀스 리셋
            return


        angle_out, speed_out, steer_rad = self._convert(angle_i, speed_i)

        msg = Float32MultiArray()
        msg.data = [float(angle_out), float(speed_out)]
        self.pub.publish(msg)

        now = time.time()
        if now - self.last_print >= self.args.throttle_sec:
            self.get_logger().info(
                f"from={addr[0]}:{addr[1]} seq={seq} pure_angle = {angle_i:.4f} angle={angle_out:.2f} speed={speed_out:.2f}"
            )
            self.last_print = now

    def destroy_node(self):
        try:
            self.sock.close()
        except Exception:
            pass
        super().destroy_node()


def build_argparser():
    p = argparse.ArgumentParser(description="ROS2 UDP Ackermann receiver -> xycar_motor publisher")
    p.add_argument("--bind-ip", default="0.0.0.0", help="Local bind IP")
    p.add_argument("--bind-port", type=int, default=5555, help="Local bind port")
    p.add_argument("--angle-scale", type=float, default=50.0)
    p.add_argument("--angle-invert", action="store_true")
    p.add_argument("--speed-scale", type=float, default=0.1)
    p.add_argument("--angle-unit", choices=["deg", "rad"], default="rad")
    p.add_argument("--angle-limit", type=float, default=50.0)
    p.add_argument("--speed-limit", type=float, default=100.0)
    p.add_argument("--topic", default="xycar_motor")
    p.add_argument("--throttle-sec", type=float, default=0.2)
    p.add_argument("--poll-sec", type=float, default=0.01)
    return p


def main(argv=None) -> int:
    filtered = remove_ros_args(sys.argv)
    args = build_argparser().parse_args(filtered[1:])
    rclpy.init(args=None)
    node = UdpAckermannReceiver(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[RC-UDP-RECV] interrupted, exiting", file=sys.stderr)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
