#!/usr/bin/env bash
# 启动 usb2can 收发测试脚本（自动剥离 conda，使用系统 python3）。
#   ./test.sh                            # 默认 100 Hz, cmd=0xA5
#   ./test.sh --rate 1000 --cmd 0x5A     # 参数透传给 test_usb2can.py
set -e

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"

# 绕开 conda：ROS rclpy 是给系统 python3.10 编的，conda 的 python3.8 会失败
if [[ -n "${CONDA_PREFIX:-}" ]]; then
  echo "[test] 检测到 conda 环境 ($CONDA_DEFAULT_ENV), 临时禁用以使用系统 python3"
  PATH=$(echo "$PATH" | tr ':' '\n' | grep -v "$CONDA_PREFIX" | paste -sd:)
  unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL CONDA_PYTHON_EXE CONDA_PROMPT_MODIFIER PYTHONPATH
fi
export PATH="/usr/bin:$PATH"

if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "[test] ERROR: /opt/ros/${ROS_DISTRO}/setup.bash 不存在" >&2
  exit 1
fi
if [[ ! -f "$WS_DIR/install/setup.bash" ]]; then
  echo "[test] ERROR: 还没编译过, 请先 ./build.sh" >&2
  exit 1
fi
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"
# shellcheck disable=SC1091
source "$WS_DIR/install/setup.bash"

exec /usr/bin/python3 "$WS_DIR/usb2can/scripts/test_usb2can.py" "$@"
