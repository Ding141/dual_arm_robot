#!/usr/bin/env bash
# 仅运行 usb2can（不编译）。
#   ./run.sh                  # 默认 /dev/ttyACM0
#   ./run.sh /dev/ttyUSB0     # 指定串口
set -e

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 优先用 udev 固定符号链接 /dev/usb2can (避免 ACM0/ACM1 漂移)
if [[ -e /dev/usb2can ]]; then
  DEFAULT_DEVICE=/dev/usb2can
else
  DEFAULT_DEVICE=/dev/ttyACM0
fi
DEVICE="${1:-$DEFAULT_DEVICE}"
ROS_DISTRO="${ROS_DISTRO:-humble}"

# 绕开 conda 环境, 使用系统 python (ROS launch/cli 依赖)
if [[ -n "${CONDA_PREFIX:-}" ]]; then
  echo "[run] 检测到 conda 环境 ($CONDA_DEFAULT_ENV), 临时禁用以使用系统 python3"
  PATH=$(echo "$PATH" | tr ':' '\n' | grep -v "$CONDA_PREFIX" | paste -sd:)
  unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL CONDA_PYTHON_EXE CONDA_PROMPT_MODIFIER PYTHONPATH
fi
export PATH="/usr/bin:$PATH"

if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "[run] ERROR: /opt/ros/${ROS_DISTRO}/setup.bash 不存在" >&2
  exit 1
fi
if [[ ! -f "$WS_DIR/install/setup.bash" ]]; then
  echo "[run] ERROR: 还没编译过, 请先执行 ./build.sh" >&2
  exit 1
fi

# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"
# shellcheck disable=SC1091
source "$WS_DIR/install/setup.bash"

# 串口可用性提示（不影响启动, 节点会自动 2s 重连）
if [[ ! -e "$DEVICE" ]]; then
  echo "[run] WARN: $DEVICE 不存在, 列出当前可用 tty:"
  ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || echo "  (无)"
  echo "[run] 节点仍会启动并每 2 秒重连一次..."
fi
if [[ -e "$DEVICE" && ! -r "$DEVICE" ]]; then
  echo "[run] WARN: 当前用户对 $DEVICE 无读权限, 可执行:"
  echo "  sudo usermod -aG dialout $USER && newgrp dialout"
fi

exec ros2 launch usb2can usb2can.launch.py device:="$DEVICE"
