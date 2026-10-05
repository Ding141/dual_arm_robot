#!/usr/bin/env python3
"""
DM 三通道电机反馈监控. 同屏刷新 24 个电机的 pos/vel/tor/T 状态.

用法:
  ros2 run usb2can motor_monitor.py                # 默认所有 channel/motor
  ros2 run usb2can motor_monitor.py --rate 20      # 刷新 20 Hz
  ros2 run usb2can motor_monitor.py --hide-empty   # 隐藏全 0 (未上线) 的电机行
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from usb2can.msg import MotorStateArray


ANSI_CLEAR = "\033[2J\033[H"
ANSI_DIM   = "\033[2m"
ANSI_BOLD  = "\033[1m"
ANSI_OK    = "\033[32m"
ANSI_WARN  = "\033[33m"
ANSI_ERR   = "\033[31m"
ANSI_RESET = "\033[0m"


class MotorMonitor(Node):
    def __init__(self, rate_hz: float, hide_empty: bool):
        super().__init__("motor_monitor")
        self.hide_empty = hide_empty
        self.last = {1: None, 2: None, 3: None}    # ch -> MotorStateArray

        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        for ch in (1, 2, 3):
            self.create_subscription(
                MotorStateArray, f"motor/ch{ch}/state",
                lambda msg, c=ch: self._on_msg(c, msg), qos)

        self.create_timer(1.0 / rate_hz, self._render)
        self.last_rx = {1: 0.0, 2: 0.0, 3: 0.0}

    def _on_msg(self, ch, msg):
        self.last[ch] = msg
        self.last_rx[ch] = time.monotonic()

    def _is_empty(self, m):
        # bridge 在没有数据时解码出 pos=-pos_max, vel=-vel_max, tor=-tor_max
        return (m.position == -12.5 and m.velocity == -30.0 and m.torque == -10.0
                and m.t_mos == 0 and m.t_rotor == 0 and m.err == 0)

    def _render(self):
        now = time.monotonic()
        out = [ANSI_CLEAR + ANSI_BOLD +
               "DM Motor Monitor  (Ctrl+C 退出)" + ANSI_RESET]
        out.append("")
        header = f"{'Ch':>3} {'Id':>3} {'CAN':>4}  {'pos[rad]':>10} {'vel[rad/s]':>11} {'tor[N·m]':>10} {'Tmos':>5} {'Tror':>5} {'err':>4}"
        for ch in (1, 2, 3):
            age = now - self.last_rx[ch] if self.last_rx[ch] else 99
            state = self.last[ch]
            color = ANSI_OK if age < 0.5 else (ANSI_WARN if age < 2 else ANSI_ERR)
            out.append(f"{color}── Channel {ch} ── 最近一帧: {age:5.2f}s 前{ANSI_RESET}")
            if state is None:
                out.append(ANSI_DIM + "  (未收到任何反馈)" + ANSI_RESET)
                out.append("")
                continue
            out.append(header)
            for i, m in enumerate(state.motors):
                empty = self._is_empty(m)
                if empty and self.hide_empty:
                    continue
                line = (f"{ch:>3} {i:>3} {i+1:>4}  "
                        f"{m.position:>10.3f} {m.velocity:>11.3f} {m.torque:>10.3f} "
                        f"{m.t_mos:>5d} {m.t_rotor:>5d} {m.err:>4d}")
                if empty:
                    out.append(ANSI_DIM + line + "  (no data)" + ANSI_RESET)
                else:
                    out.append(line)
            out.append("")
        sys.stdout.write("\n".join(out))
        sys.stdout.flush()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--rate", type=float, default=10.0, help="屏幕刷新频率 Hz")
    parser.add_argument("--hide-empty", action="store_true",
                        help="隐藏未上线 (全 0 解码) 的电机行")
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = MotorMonitor(args.rate, args.hide_empty)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
