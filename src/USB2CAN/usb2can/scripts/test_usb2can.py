#!/usr/bin/env python3
"""
usb2can 收发测试脚本。

用法（先编译并 source 工作区）:
    ./build.sh
    source install/setup.bash
    python3 usb2can/scripts/test_usb2can.py
    # 或:
    python3 usb2can/scripts/test_usb2can.py --rate 50 --cmd 0xA5

功能:
  - 以指定频率向 /dcu/command 发布测试帧
    (CTRL1/2/3 的 cmd=参数, payload 前 8 字节填递增计数, 其余 0)
  - 订阅 /imu/data, 打印加速度/角速度/姿态
  - 订阅 /dcu/feedback, 打印 CTRL1 前 16 字节(ID0/ID1 槽位) 以及 RX 帧率
"""
import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Imu
from usb2can.msg import DcuCommand, DcuFeedback, CanFdChannelCmd


def make_channel_cmd(cmd: int, counter: int) -> CanFdChannelCmd:
    ch = CanFdChannelCmd()
    ch.cmd = cmd & 0xFF
    payload = [0] * 64
    # 前 8 字节填 counter (小端) + 标识
    payload[0] = counter & 0xFF
    payload[1] = (counter >> 8) & 0xFF
    payload[2] = (counter >> 16) & 0xFF
    payload[3] = (counter >> 24) & 0xFF
    payload[4] = 0xDE
    payload[5] = 0xAD
    payload[6] = 0xBE
    payload[7] = 0xEF
    ch.payload = payload
    return ch


class TestNode(Node):
    def __init__(self, rate_hz: float, cmd_byte: int):
        super().__init__('usb2can_tester')
        self.cmd_byte = cmd_byte
        self.counter = 0
        self.rx_count = 0
        self.rx_window_start = time.monotonic()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.pub_cmd = self.create_publisher(DcuCommand, '/dcu/command', 10)
        self.sub_imu = self.create_subscription(Imu, '/imu/data', self.on_imu, sensor_qos)
        self.sub_fb  = self.create_subscription(DcuFeedback, '/dcu/feedback', self.on_feedback, sensor_qos)

        period = 1.0 / rate_hz if rate_hz > 0 else 0.1
        self.tx_timer = self.create_timer(period, self.on_tx)

        self.get_logger().info(
            f'Tester started: tx_rate={rate_hz} Hz, cmd=0x{cmd_byte:02X}')

    # ---------- TX ----------
    def on_tx(self):
        msg = DcuCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'usb2can_tester'
        msg.ctrl1 = make_channel_cmd(self.cmd_byte, self.counter)
        msg.ctrl2 = make_channel_cmd(self.cmd_byte, self.counter)
        msg.ctrl3 = make_channel_cmd(self.cmd_byte, self.counter)
        msg.imu_cmd = 0x01
        self.pub_cmd.publish(msg)
        self.counter = (self.counter + 1) & 0xFFFFFFFF

    # ---------- RX ----------
    def on_imu(self, msg: Imu):
        a = msg.linear_acceleration
        g = msg.angular_velocity
        q = msg.orientation
        # 控制台不要刷得太凶
        self.get_logger().info(
            f'IMU acc=({a.x:+.2f},{a.y:+.2f},{a.z:+.2f}) '
            f'gyr=({g.x:+.2f},{g.y:+.2f},{g.z:+.2f}) '
            f'quat=({q.w:+.2f},{q.x:+.2f},{q.y:+.2f},{q.z:+.2f})',
            throttle_duration_sec=0.5,
        )

    def on_feedback(self, msg: DcuFeedback):
        self.rx_count += 1
        now = time.monotonic()
        elapsed = now - self.rx_window_start
        if elapsed >= 1.0:
            hz = self.rx_count / elapsed
            self.rx_count = 0
            self.rx_window_start = now
            c1 = bytes(msg.ctrl1[:16]).hex(' ')
            self.get_logger().info(f'RX rate {hz:6.1f} Hz | CTRL1[0..15]= {c1}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rate', type=float, default=100.0, help='TX 频率 Hz (默认 100)')
    ap.add_argument('--cmd', type=lambda x: int(x, 0), default=0xA5,
                    help='下发 Cmd 字节, 支持 0x.. 形式 (默认 0xA5)')
    args = ap.parse_args()

    rclpy.init()
    node = TestNode(args.rate, args.cmd)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
