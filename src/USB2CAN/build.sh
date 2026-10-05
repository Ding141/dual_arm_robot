#!/usr/bin/env bash
# 仅编译 usb2can。
#   ./build.sh           # 增量编译
#   CLEAN=1 ./build.sh   # 清空 build/install/log 后重编
set -e

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"

# ---- 绕开 conda 环境 ----
# ROS Humble 的 rosidl_* 工具装在系统 python3.10, 走 conda 的 python 会缺少
# em/lark 等模块导致 rosidl_adapter 失败。这里强制使用系统 python 编译。
if [[ -n "${CONDA_PREFIX:-}" ]]; then
  echo "[build] 检测到 conda 环境 ($CONDA_DEFAULT_ENV), 临时禁用以使用系统 python3"
  # 把 conda 的 bin 路径从 PATH 里剔除
  PATH=$(echo "$PATH" | tr ':' '\n' | grep -v "$CONDA_PREFIX" | paste -sd:)
  unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL CONDA_PYTHON_EXE CONDA_PROMPT_MODIFIER PYTHONPATH
fi
export PATH="/usr/bin:$PATH"

if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "[build] ERROR: /opt/ros/${ROS_DISTRO}/setup.bash 不存在" >&2
  exit 1
fi
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"

cd "$WS_DIR"

if [[ "${CLEAN:-0}" == "1" ]]; then
  echo "[build] CLEAN=1 -> rm -rf build install log"
  rm -rf build install log
fi

# 修复 --symlink-install 与之前非 symlink 构建残留冲突
if [[ -d "build/usb2can/ament_cmake_python/usb2can/usb2can" && \
      ! -L "build/usb2can/ament_cmake_python/usb2can/usb2can" ]]; then
  echo "[build] 检测到 ament_cmake_python 残留目录，自动清理"
  rm -rf build/usb2can install/usb2can
fi

SYS_PY=/usr/bin/python3
echo "[build] colcon build (PYTHON_EXECUTABLE=$SYS_PY)"
colcon build --packages-select usb2can --symlink-install \
  --cmake-args "-DPYTHON_EXECUTABLE=$SYS_PY"

echo "[build] 完成。运行节点: ./run.sh [/dev/ttyACM0]"
