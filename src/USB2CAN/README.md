# USB2CAN — STM32 DCU 桥接 + DM4310 上层封装

ROS 2 Humble 工作区。把一颗 STM32 DCU 透过 USB CDC 接到主机，再在上层用 MIT 模式
独立控制达秒 (DAMIAO) DM4310 系列电机。

```
                ┌──────────────────────┐  /motor/ch{1,2,3}/state
                │  dm_motor_bridge     │ ────────────────────────▶  上层算法
                │  (DM4310 MIT 解析)    │ ◀────────────────────────  下发指令
                └──────────┬───────────┘  /motor/ch{1,2,3}/cmd
                           │

`MotorEnableArray` 内部会根据 `enable` 字段自动选 `FC`（使能）或 `FD`（失能）。
              /dcu/feedback│/dcu/command   /imu/data
                           ▼
                ┌──────────────────────┐
                │  usb2can_node        │ ── /dev/ttyACM0 ─▶ STM32 DCU
                │  (245B 定长帧 + CRC) │ ◀────────────────  3×CANFD + IMU
                └──────────────────────┘
```

---

## 目录速览

| 路径 | 作用 |
|------|------|
| `build.sh` | 一键编译（自动绕开 conda、修 symlink-install 残留） |
| `run.sh`   | 启动 `usb2can_node`（裸的串口桥） |
| `test.sh`  | 启动 Python 收发测试脚本 |
| `usb2can/` | ROS 2 包：消息定义 / 节点源码 / launch / config |
| `usb2can/src/usb2can_node.cpp` | 串口桥（245B 帧 / CRC / 低延迟） |
| `usb2can/src/dm_motor_bridge.cpp` | DM4310 MIT 上层桥 |
| `usb2can/scripts/test_usb2can.py` | 双向收发压测脚本 |
| `usb2can/launch/usb2can_with_dm.launch.py` | **单板** 启动（usb2can_node + dm_motor_bridge） |
| `usb2can/launch/usb2can_dual.launch.py`    | **双板** 启动（两组节点 + namespace 隔离，详见 §3.9） |

---

## 🚀 快速上手 (TL;DR)

> 完整流程：从启动节点 → 使能电机 → 发送 MIT 控制 → 失能关电。
> 假设你要控制 **ch1 上的 motor 0 和 motor 2**（CAN ID = 1 和 3）。

### 步骤 1 — 编译（首次或代码改动后）

```bash
cd ~/USB2CAN

`MotorEnableArray` 内部会根据 `enable` 字段自动选 `FC`（使能）或 `FD`（失能）。
./build.sh
```

### 步骤 2 — 启动桥接节点（**终端 A**，保持运行）

```bash
source install/setup.bash
ros2 launch usb2can usb2can_with_dm.launch.py device:=/dev/ttyACM0
```

> 🔌 **接两块 STM32 板？** 见 [§3.9 双板 / 多板部署](#39-双板--多板部署)。
> 用 `ros2 launch usb2can usb2can_dual.launch.py` 起两组节点，话题自动按 `/boardA/...` `/boardB/...` 隔离。

看到这行才算就绪：
```

`MotorEnableArray` 内部会根据 `enable` 字段自动选 `FC`（使能）或 `FD`（失能）。
[dm_motor_bridge-2] DM bridge ready (pos±12.5 vel±30.0 tor±10.0 Kp 0..500 Kd 0..5.0)
[usb2can_node-1]   Opened /dev/ttyACM0 @ 921600 baud (low-latency)
```

### 步骤 3 — 周期下发 MIT 控制（500 Hz）

> 🎉 **不需要先发 DM 使能命令，也不需要 enable**（STM32 端按 cmd 字节位自动使能）。
> 详见 §10 的"STM32 端自动使能设计"。如果你的固件还没实现自动使能，看 §10 末尾的 [手动使能流程]。

```bash
ros2 topic pub -r 500 /motor/ch1/cmd usb2can/msg/MotorCommandArray "{
  channel: 1,
  motors: [
    {id: 0, position:  1.57, velocity: 0.0, kp: 10.0, kd: 1.0, torque_ff: 0.0},
    {id: 2, position: -0.50, velocity: 0.0, kp: 10.0, kd: 1.0, torque_ff: 0.0}
  ]
}"
```

⚠️ **必须 `-r 500` 周期发**，不能用 `--once`。MIT 模式靠周期帧维持力矩；停发后电机看门狗会进入保护态。

### 步骤 4 — 观察反馈（**新开终端 C**）

桥接节点已经按 **DM RX 协议** 把每通道的 64B 反馈拆成 8 个 `MotorState`：
位置/速度/力矩/MOS 温度/转子温度/错误码。直接订阅：

```bash
source ~/USB2CAN/install/setup.bash

# 单通道
ros2 topic echo /motor/ch1/state
ros2 topic echo /motor/ch2/state
ros2 topic echo /motor/ch3/state

# 只看某一颗电机
ros2 topic echo /motor/ch1/state --field motors[0]

# 反馈速率
ros2 topic hz /motor/ch1/state
```

**三通道同屏汇总**（推荐，刷新式显示 24 个电机一目了然）：

```bash
ros2 run usb2can motor_monitor.py                  # 默认 10 Hz 刷新
ros2 run usb2can motor_monitor.py --rate 20
ros2 run usb2can motor_monitor.py --hide-empty     # 隐藏未上线 (全 0) 的行
```

> 💡 如果你看到反馈全是 `pos=-12.5 vel=-30 tor=-10`，说明 STM32 传回来的那 64 字节是 0（电机没接 / 没响应 / STM32 还没把 CAN 反馈塞进去）。这是「无数据」的特征值，不是 bug。`motor_monitor.py` 会把这些行标灰并显示 `(no data)`。

### 步骤 5 — 关电

**正常关电**：直接 `Ctrl+C` 终端 B 的 `-r 500` 命令。

主机停发 → STM32 看门狗超时（>100ms）→ 自动给原本 `bit=1` 的位发 `DM_DISABLE`（前提：固件实现了 §10 的看门狗）。

**紧急停（不依赖 watchdog，立即生效）**：

```bash
ros2 topic pub --once /motor/ch1/enable std_msgs/msg/Bool "{data: false}"
# -> 立即一帧 cmd=0x00, STM32 看到下降沿对所有 prev_mask 为 1 的位发 DM_DISABLE
```

最后 `Ctrl+C` 终端 A 关闭桥接节点。

### 速查：DM 特殊命令字节

| 8 字节 data | 含义 |
|------------|------|
| `255,255,255,255,255,255,255,252` (`FC`) | **使能**（开启 MIT 模式） |
| `255,255,255,255,255,255,255,253` (`FD`) | **失能** |
| `255,255,255,255,255,255,255,251` (`FB`) | 清除错误 |
| `255,255,255,255,255,255,255,254` (`FE`) | 保存零位 |

### 槽位 ↔ CAN ID 映射

| bridge 里的 `id` | 实际 CAN ID | payload 字节范围 |
|------------------|-------------|------------------|
| `0` | `1` | `[ 0..7]` |
| `1` | `2` | `[ 8..15]` |
| `2` | `3` | `[16..23]` |
| `3` | `4` | `[24..31]` |
| `4` | `5` | `[32..39]` |
| `5` | `6` | `[40..47]` |
| `6` | `7` | `[48..55]` |
| `7` | `8` | `[56..63]` |

---

## 0. 系统要求

- Ubuntu 22.04 + ROS 2 **Humble**
- 用户在 `dialout` 组里（一次性）：

  ```bash
  sudo usermod -aG dialout $USER && newgrp dialout
  ```

- ⚠️ **不要在 conda 环境里跑 ROS**。脚本会自动剥离 conda；直接用 `ros2`/`python3` 时
  请先 `conda deactivate`。

---

## 1. 编译

```bash
cd ~/USB2CAN
./build.sh                 # 增量
CLEAN=1 ./build.sh         # 清空 build/install/log 重编
```

特性：
- 自动从 `PATH` 摘除 `CONDA_PREFIX`，强制用系统 `python3`
- 检测并清理上次失败留下的 `ament_cmake_python` 残留目录
- 显式 `--cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3`

---

## 2. 运行

### 2.1 只启动串口桥（不带 DM 解析层）

```bash
./run.sh                   # 默认 /dev/ttyACM0
./run.sh /dev/ttyUSB0      # 指定设备
```

启动后暴露：

| 话题 | 类型 | 方向 |
|------|------|------|
| `/dcu/command`  | `usb2can/msg/DcuCommand`  | 订阅（裸 245B 下行） |
| `/dcu/feedback` | `usb2can/msg/DcuFeedback` | 发布（裸 245B 上行） |
| `/imu/data`     | `sensor_msgs/msg/Imu`     | 发布 |

USB 拔插时节点不会退出，**每 2 秒自动重连一次**。

### 2.2 启动串口桥 + DM4310 MIT 桥（推荐）

```bash
source install/setup.bash
ros2 launch usb2can usb2can_with_dm.launch.py device:=/dev/ttyACM0
```

这会一次起两个节点：`usb2can_node` 和 `dm_motor_bridge`。

---

## 3. DM4310 MIT 桥使用

`dm_motor_bridge` 把 3 个 CTRL 通道里的每个 64B payload 切成 8 槽位 × 8B，
按 DM 厂家 MIT 协议自动 pack/unpack。

### 3.1 话题接口（每通道 N=1/2/3 一份）

| Topic | 类型 | 方向 | 说明 |
|-------|------|------|------|
| `/motor/chN/state`     | `usb2can/MotorStateArray`   | pub | 8 个电机的位置/速度/力矩/温度/状态 |
| `/motor/chN/cmd`       | `usb2can/MotorCommandArray` | sub | MIT 模式独立控制（自动 mask） |
| `/motor/chN/enable`    | `std_msgs/Bool`             | sub | 该通道总开关，false 时不转发 |
| `/motor/chN/raw_cmd`   | `usb2can/RawCanFrameArray`  | sub | 一次性原始 8B CAN 帧（任意字节透传：清错/保存零位等） |
| `/motor/chN/motor_enable` | `usb2can/MotorEnableArray` | sub | **逐颗电机**使能/失能（DM `FC`/`FD` 的友好封装） |

### 3.2 `Cmd` 字节语义（与 STM32 端约定）

纯 **bitmask** 语义，不保留 DM 广播路径（安全考虑：避免一帧误打总线上未被本桥管理的设备）：

| 值 | 含义 |
|----|------|
| `0x00` | 该通道不转发任何 CAN 报文 |
| `0x01..0xFF` | **bitmask**：bit `i` = 1 表示 payload 第 `i*8 .. i*8+7` 字节作为 1 帧 8B CAN 报文转发，CAN ID = `i+1`。`0xFF` = 8 位全 1，STM32 同时点对点发 8 帧到 ID=1..8 |

`dm_motor_bridge` 自动按 `MotorCommandArray.motors[]` 出现的电机 id 求 OR 得到 mask，直接透传（包括 8 颗都填时的 `0xFF`）。

> ⚠ 需 STM32 固件配合：看到 `cmd=0xFF` 也当 bitmask 处理，不走“CAN ID=0 广播 CANFD”路径。详见§10。

### 3.3 三步把电机转起来（**显式使能**工作流）

```bash
# 第 1 步: 启动桥
ros2 launch usb2can usb2can_with_dm.launch.py device:=/dev/ttyACM0

# 第 2 步: 使能 ch1 上要用的电机 (motor 0 和 motor 2)
#   bridge 会一帧把这两颗的 8B payload 填成 FF FF FF FF FF FF FF FC,
#   cmd 字节 = 0x05, STM32 转发到 CAN ID=1 和 3, 电机进入 MIT 模式.
ros2 topic pub -r 1 /motor/ch2/motor_enable usb2can/msg/MotorEnableArray "{
  channel: 2,
  motors: [
    {id: 0, enable: true},
    {id: 1, enable: true},
    {id: 2, enable: true},
    {id: 3, enable: true},
    {id: 4, enable: true},
    {id: 5, enable: true},
    {id: 6, enable: true},
    {id: 7, enable: true}
  ]
}"

ros2 topic pub -r 1 /motor/ch1/motor_enable usb2can/msg/MotorEnableArray "{
  channel: 1,
  motors: [
    {id: 0, enable: false},
    {id: 1, enable: false},
    {id: 2, enable: false},
    {id: 3, enable: false},
    {id: 4, enable: false},
    {id: 5, enable: false},
    {id: 6, enable: false},
    {id: 7, enable: false}
  ]
}"

# 第 3 步: 周期下发 MIT 控制 (300 Hz)
ros2 topic pub -r 10 /motor/ch1/cmd usb2can/msg/MotorCommandArray "{
  channel: 1,
  motors: [
    {id: 0, position:  0.00, velocity: 0.0, kp: 0.0, kd: 0.0, torque_ff: 0.0},
    {id: 1, position:  0.00, velocity: 0.0, kp: 0.0, kd: 0.0, torque_ff: 0.0},
    {id: 2, position:  0.00, velocity: 0.0, kp: 0.0, kd: 0.0, torque_ff: 0.0},
    {id: 3, position:  0.00, velocity: 0.0, kp: 0.0, kd: 0.0, torque_ff: 0.0},
    {id: 4, position:  0.00, velocity: 0.0, kp: 0.0, kd: 0.0, torque_ff: 0.0},
    {id: 5, position:  0.00, velocity: 0.0, kp: 0.0, kd: 0.0, torque_ff: 0.0},
    {id: 6, position:  0.00, velocity: 0.0, kp: 0.0, kd: 0.0, torque_ff: 0.0},
    {id: 7, position:  0.00, velocity: 0.0, kp: 0.0, kd: 0.0, torque_ff: 0.0}
  ]
}"
```

> 此时周期帧 `Cmd = 0b00000101 = 0x05`，STM32 把 motor 0/2 的 8B MIT 命令各自
> 转成一帧 CAN（CAN ID 分别为 1 和 3）。motor 1,3..7 的字节虽然存在但 bit=0，不转发。

**关电时反向操作**：

```bash
# 1) Ctrl+C 终端里的 -r 500 cmd
# 2) 失能这两颗电机
ros2 topic pub --once /motor/ch1/motor_enable usb2can/msg/MotorEnableArray "{
  channel: 1,
  motors: [
    {id: 0, enable: false},
    {id: 2, enable: false}
  ]
}"
```

`MotorEnableArray` 内部会根据 `enable` 字段自动选 `FC`（使能）或 `FD`（失能）。
**一帧 DcuCommand 就完成多颗电机的 enable/disable**，无需手工拼 raw 字节。

**一键开/关整条总线**

协议上 `cmd=0xFF` 现在不再是 DM 广播，而是 "bitmask 全 1" —— STM32 会 **同时发 8 帧点对点** CAN 到 ID=1..8，每帧 8B。于是一帧 DcuCommand 就能让总线上 8 颗电机同时使能，而且不会误打到未被本桥管理的设备。

```bash
# 打开 ch1 上 8 颗电机 (cmd=0xFF, payload = 8 份 FF..FC, 点对点 8 帧 CAN)
ros2 topic pub --once /motor/ch1/motor_enable usb2can/msg/MotorEnableArray "{
  channel: 1,
  motors: [
    {id: 0, enable: true}, {id: 1, enable: true},
    {id: 2, enable: true}, {id: 3, enable: true},
    {id: 4, enable: true}, {id: 5, enable: true},
    {id: 6, enable: true}, {id: 7, enable: true}
  ]
}"

# 全失能同理 (enable: false), payload 变成 8 份 FF..FD。
```

> 每颗电机只接收点对点的 `FF FF FF FF FF FF FF FC` / `FD`（CAN ID = `id+1`），未被本桥管理的设备不会被波及。

### 3.4 反馈读取与解析（三通道）

`dm_motor_bridge` 收到 `/dcu/feedback` 后，把每个通道的 **64 字节** payload 切成
**8 个槽位 × 8 字节**，每个槽位按 **DM RX 协议**自动解码成一个 `MotorState`：

```
B0 = (err << 4) | id           # 高 4 位错误码, 低 4 位电机 id
B1 = pos[15:8]                 # 16-bit pos -> [-pos_max, +pos_max]
B2 = pos[7:0]
B3 = vel[11:4]                 # 12-bit vel -> [-vel_max, +vel_max]
B4 = (vel[3:0]<<4) | t[11:8]   # 12-bit torque -> [-tor_max, +tor_max]
B5 = t[7:0]
B6 = T_mos     (°C, uint8)
B7 = T_rotor   (°C, uint8)
```

解码后发布到三个 topic（QoS=`best_effort`, 与 IMU 同档）：

| Topic | 类型 |
|-------|------|
| `/motor/ch1/state` | `usb2can/MotorStateArray` (8 个 `MotorState`) |
| `/motor/ch2/state` | 同上 |
| `/motor/ch3/state` | 同上 |

每个 `MotorState` 字段：

```
uint8   id          # 槽位 0..7 (CAN ID = id + 1)
float32 position    # rad
float32 velocity    # rad/s
float32 torque      # N·m
uint8   t_mos       # °C
uint8   t_rotor     # °C
uint8   err         # DM 错误码 0..15
```

#### A) 命令行直接看

```bash
# 单通道完整反馈
ros2 topic echo /motor/ch1/state
ros2 topic echo /motor/ch2/state
ros2 topic echo /motor/ch3/state

# 只看某一颗电机
ros2 topic echo /motor/ch1/state --field motors[0]
ros2 topic echo /motor/ch2/state --field motors[3]

# 反馈速率
ros2 topic hz /motor/ch1/state
```

#### B) 三通道同屏汇总（推荐）

```bash
ros2 run usb2can motor_monitor.py                  # 默认 10 Hz 刷新
ros2 run usb2can motor_monitor.py --rate 20
ros2 run usb2can motor_monitor.py --hide-empty     # 隐藏未上线 (全 0) 行
```

显示：

```
DM Motor Monitor  (Ctrl+C 退出)

── Channel 1 ── 最近一帧:  0.01s 前
 Ch  Id  CAN    pos[rad]  vel[rad/s]   tor[N·m]  Tmos  Tror  err
  1   0    1       0.123       0.005      0.012    28    32    0
  1   1    2      (no data)
  ...
── Channel 2 ── 最近一帧:  0.02s 前
  ...
```

颜色提示：绿 = 新鲜 (<0.5 s)，黄 = >0.5 s，红 = >2 s 没收到。

#### C) 在你自己的 ROS 2 节点里订阅（Python 示例）

```python
import rclpy
from rclpy.node import Node
from usb2can.msg import MotorStateArray, MotorCommandArray, MotorCommand

class MyController(Node):
    def __init__(self):
        super().__init__('my_controller')
            MotorCommandArray, '/motor/ch1/cmd', 10)
        self.create_timer(0.002, self.tick)        # 500 Hz
        # 订阅 ch1 反馈
        self.create_subscription(
            MotorStateArray, '/motor/ch1/state', self.on_state, 10)
        # 发布 ch1 控制
        self.cmd_pub = self.create_publisher(
            MotorCommandArray, '/motor/ch1/cmd', 10)
        self.create_timer(0.002, self.tick)        # 500 Hz

    def on_state(self, msg: MotorStateArray):
        m0 = msg.motors[0]                          # CAN ID=1 的电机
        self.get_logger().info(
            f'ch1.m0 pos={m0.position:.3f} vel={m0.velocity:.3f} '
            f'tor={m0.torque:.3f} T={m0.t_mos}°C err={m0.err}')

    def tick(self):
rclpy.init(); rclpy.spin(MyController()); rclpy.shutdown()
        cmd = MotorCommandArray(channel=1)
        cmd.motors = [MotorCommand(
            id=0, position=1.57, velocity=0.0,
            kp=10.0, kd=1.0, torque_ff=0.0)]
        self.cmd_pub.publish(cmd)

rclpy.init(); rclpy.spin(MyController()); rclpy.shutdown()
```

C++ 用 `#include "usb2can/msg/motor_state_array.hpp"` 同理。

#### ⚠️ 看到 `pos=-12.5 vel=-30 tor=-10` 是什么意思？

这是 STM32 把该槽位 8 字节填了 **全 0** 的解码结果。原因可能是：
- 没有真实电机挂在那个 CAN ID 上
- 电机没回复（断电/没使能/线缆未接好）
- STM32 端固件还没把 CAN 反馈写入对应 64B 缓冲

链路本身是通的（你能看到帧在持续发布、`ros2 topic hz` 有数字）。
`motor_monitor.py` 会自动识别这种"特征值"并标灰显示 `(no data)`。

真实电机回包正常时，典型数值类似：

```
pos ≈ 0 ± 实际角度    vel ≈ 0    tor ≈ 0    T_mos ≈ 25-50°C    err = 0
```

### 3.5 失能

```bash
ros2 topic pub --once /motor/ch1/enable std_msgs/msg/Bool "{data: false}"
```

立刻发一帧 `Cmd=0`。

### 3.6 为什么不再提供 `/motor/chN/broadcast` 跟 DM 广播

原本设计里 `cmd=0xFF` 表示“64B 当一帧 CAN-FD 广播到 ID=0”，嵌在未管理的设备上有误接不可接受的安全风险（例如总线上接了另外一套设备会一起响应）。现在重定义：

- `cmd=0xFF` = bitmask 全 1（同时点对点发 8 帧到 ID=1..8）
- 不再保留 “ID=0 64B CANFD” 路径
- 原 `/motor/chN/broadcast` topic **已删除**

如果你确实需要发任意 64B 原始字节到 STM32，请用 `/dcu/command` 直发 `DcuCommand`，或用 `/motor/chN/raw_cmd` 逐槽位填 8B。

### 3.7 节点参数

`dm_motor_bridge` 全部参数从 `config/dm_motors.yaml` 读取（见 §9）。汇总：

| 参数 | 默认 | 说明 |
|------|------|------|
| `pos_max` | `12.5` | 位置范围 ±（rad）— **全局默认**，可被 per-motor override |
| `vel_max` | `30.0` | 速度范围 ±（rad/s） |
| `tor_max` | `10.0` | 力矩范围 ±（N·m） |
| `kp_max`  | `500`  | Kp 上限 |
| `kd_max`  | `5`    | Kd 上限 |
| `enable_chN` | `true` | 是否在 chN 上开放 state/cmd/enable/raw_cmd/motor_enable 话题 |
| `default_enable_chN` | `true` | 启动时该通道总开关（true = cmd 字节 = slot_mask，详见 §10） |
| `ch{N}.id{M}.pos_max` 等 | 继承全局 | 单颗电机限幅 override（见 §9） |
| `ch{N}.id{M}.type` | `""` | 电机型号字符串（仅作日志） |
| `imu_cmd` | `0` | 透传到下行帧的 IMU CMD 字节 |

> 单台机器人多种型号电机混用时**必须**配 `dm_motors.yaml`，否则不同型号的限幅
> 不一致会导致 MIT 编解码错位。详见 §9.6。

### 3.8 消息类型参考

| 消息 | 用途 |
|------|------|
| `usb2can/msg/DcuCommand`        | 下行 245B 帧的逻辑结构（host → STM32） |
| `usb2can/msg/DcuFeedback`       | 上行 245B 帧的逻辑结构（STM32 → host） |
| `usb2can/msg/CanFdChannelCmd`   | 1 字节 cmd + 64B payload，是 DcuCommand 里 3 个通道的容器 |
| `usb2can/msg/MotorCommand`      | 单颗电机的 MIT 控制（pos/vel/kp/kd/torque_ff） |
| `usb2can/msg/MotorCommandArray` | 一个通道里若干个 `MotorCommand`，自动 OR 出 cmd mask |
| `usb2can/msg/MotorState`        | 单颗电机的 MIT 反馈（pos/vel/torque/温度/err） |
| `usb2can/msg/MotorStateArray`   | 一个通道里固定 8 个 `MotorState`（按槽位 0..7） |
| `usb2can/msg/RawCanFrame`       | 一帧 8B 原始 CAN（任意字节透传，高级用户用） |
| `usb2can/msg/RawCanFrameArray`  | 一个通道里若干个 `RawCanFrame`，一次性脉冲发出 |
| `usb2can/msg/MotorEnable`       | 单颗电机的使能/失能请求（自动翻译成 DM `FC`/`FD`） |
| `usb2can/msg/MotorEnableArray`  | 一个通道里若干颗电机的 enable/disable，一次合并发出 |

#### 完整字段定义

**`@/home/ubuntu/USB2CAN/usb2can/msg/CanFdChannelCmd.msg`**

```
# One CAN-FD channel command (Cmd + 64-byte payload).
# Layout in the on-wire frame: 1 byte cmd + 64 bytes payload = 65 bytes.
uint8 cmd
uint8[64] payload
```

**`@/home/ubuntu/USB2CAN/usb2can/msg/DcuCommand.msg`**

```
# Down-link command frame to the DCU (host -> STM32).
std_msgs/Header header
CanFdChannelCmd  ctrl1
CanFdChannelCmd  ctrl2
CanFdChannelCmd  ctrl3
uint8            imu_cmd
```

**`@/home/ubuntu/USB2CAN/usb2can/msg/DcuFeedback.msg`**

```
# Up-link feedback frame from the DCU (STM32 -> host).
# Each CTRLx is 64 bytes laid out as 8 IDs * 8 bytes:
#   ID0 -> bytes[0..7]
#   ID1 -> bytes[8..15]
#   ...
#   ID7 -> bytes[56..63]
std_msgs/Header header
uint8[64] ctrl1
uint8[64] ctrl2
uint8[64] ctrl3
sensor_msgs/Imu imu
```

**`@/home/ubuntu/USB2CAN/usb2can/msg/MotorCommand.msg`**

```
# DM MIT 模式电机命令（单个电机）
# 物理量在 per-motor 限幅 [pos_max,vel_max,tor_max,kp_max,kd_max] 范围内, 越界会被钳位。
uint8   id          # 槽位 / 电机 ID (0..7)
float32 position    # rad     (位置环目标)
float32 velocity    # rad/s   (速度前馈)
float32 kp          # 0..kp_max
float32 kd          # 0..kd_max
float32 torque_ff   # N*m    (力矩前馈)
```

**`@/home/ubuntu/USB2CAN/usb2can/msg/MotorCommandArray.msg`**

```
# 一个 CTRL 通道下所有电机命令。
# 数组里没出现的 id (0..7) 在该帧里全部置零（pos=vel=kp=kd=tor=0）,
# 即 DM 电机 free / 力矩 0 状态。
std_msgs/Header   header
uint8             channel       # 1 / 2 / 3
MotorCommand[]    motors        # 0..8 个, 按 id 任意顺序
```

**`@/home/ubuntu/USB2CAN/usb2can/msg/MotorState.msg`**

```
# DM MIT 模式电机反馈（单个电机）
uint8   id          # 槽位 / 电机 ID  (0..7)
float32 position    # rad
float32 velocity    # rad/s
float32 torque      # N*m
uint8   t_mos       # °C  MOSFET 温度
uint8   t_rotor     # °C  转子温度
uint8   err         # 状态/错误位 (RX byte0 高 4 bit)
```

**`@/home/ubuntu/USB2CAN/usb2can/msg/MotorStateArray.msg`**

```
# 一个 CTRL 通道(8 槽位)的全部反馈
std_msgs/Header header
uint8           channel         # 1 / 2 / 3
MotorState[8]   motors          # 顺序 = 槽位 0..7
```

**`@/home/ubuntu/USB2CAN/usb2can/msg/RawCanFrame.msg`**

```
# 一帧原始 8B CAN 报文 (槽位 i -> CAN ID = i+1)
# 用于 DM 厂家"特殊命令" (使能/失能/清错/保存零位等), 也可用作任意点对点字节透传。
uint8    id         # 槽位 0..7  (实际 CAN ID = id+1)
uint8[8] data       # 8 字节原始数据
```

**`@/home/ubuntu/USB2CAN/usb2can/msg/RawCanFrameArray.msg`**

```
# 一个 CTRL 通道下若干原始 8B CAN 帧。
# bridge 会自动:
#   - 把每帧 8 字节复制到 payload[id*8 .. id*8+7]
#   - cmd 字节 = OR over (1<<id), 让 STM32 转发对应槽位
#   - 没出现的槽位会被覆盖为 0 (不会被转发, 因为对应 bit 为 0)
std_msgs/Header   header
uint8             channel       # 1 / 2 / 3
RawCanFrame[]     frames        # 0..8 个 (8 颗都填时 cmd=0xFF, STM32 同时点对点发 8 帧)
```

**`@/home/ubuntu/USB2CAN/usb2can/msg/MotorEnable.msg`**

```
# 单颗 DM 电机的使能 / 失能请求。
# bridge 会把它翻译成一帧 8B 原始 CAN 报文:
#   enable = true  -> FF FF FF FF FF FF FF FC  (DM 使能, 进入 MIT 模式)
#   enable = false -> FF FF FF FF FF FF FF FD  (DM 失能, 退出 MIT 模式)
# 一次性脉冲, 不会被周期 /motor/chN/cmd 覆盖.
uint8 id          # 槽位 / 电机 ID (0..7), 实际 CAN ID = id + 1
bool  enable      # true=使能, false=失能
```

**`@/home/ubuntu/USB2CAN/usb2can/msg/MotorEnableArray.msg`**

```
# 一个 CTRL 通道里若干颗电机的使能/失能请求.
# bridge 一次性合并成 1 帧 DcuCommand 发给 STM32:
#   - cmd 字节 = OR over (1 << id), 让 STM32 把对应槽位作为 CAN 帧转发
#   - payload[id*8 .. id*8+7] = DM_ENABLE 或 DM_DISABLE 字节序列
# 没出现在数组里的槽位不动 (对应 bit 为 0, STM32 不转发).
#
# 协议: cmd=0xFF 重定义为 bitmask 全 1, 表示同时点对点发 8 帧 CAN 到 ID=1..8.
#   不再保留原“CAN ID=0 广播 CANFD”路径, 以避免误打本桥未管理的设备.
#   一次填 8 颗 -> 8 帧点对点同时发出, 完全安全.
std_msgs/Header   header
uint8             channel       # 1 / 2 / 3
MotorEnable[]     motors        # 0..8 个
```

#### 用例对照

| 我想…… | 用哪个 topic / 消息 |
|--------|---------------------|
| 周期控制电机位置/力矩（MIT 模式） | `/motor/chN/cmd` ← `MotorCommandArray` |
| 读电机反馈（位置/速度/温度） | `/motor/chN/state` → `MotorStateArray` |
| 发 DM 使能 / 失能 / 清错 等特殊命令 | `/motor/chN/raw_cmd` ← `RawCanFrameArray` |
| 通道急停（强制 cmd=0） | `/motor/chN/enable` ← `std_msgs/Bool false` |
| 不经 DM 解析直接玩 245B 帧 | `/dcu/command` ← `DcuCommand` / `/dcu/feedback` → `DcuFeedback` |

### 3.9 双板 / 多板部署

如果你要在同一台主机上同时接 **两块（或更多）STM32 USB 板**，需要解决两件事：

1. **设备名稳定**：`/dev/ttyACM0/1` 上电先后会随机分配，必须用 udev 规则绑定为固定符号链接（如 `/dev/usb2can_a`、`/dev/usb2can_b`）。
2. **ROS 话题隔离**：每块板子启动一组 (`usb2can_node` + `dm_motor_bridge`)，各自放在独立 namespace 里，避免 `/dcu/feedback` 等话题互相覆盖。

> ✅ 节点内部所有话题都是**相对话题**，所以单板（默认根 namespace）和多板（带 `boardA/boardB` 前缀）走的是**同一份代码**，单板使用方式不变。

#### Step A — 用 udev 规则给两块板子起稳定名字

先插上**只插一块**的状态下查一下设备属性（找出能区分两块板子的字段）：

```bash
udevadm info -a -n /dev/ttyACM0 | grep -E 'ATTRS\{(serial|idVendor|idProduct|devpath)\}' | head -10
```

理想情况：两块板子 STM32 固件设了不同的 USB **序列号**。否则用 USB **物理端口位置**（`KERNELS=="1-1.2"` 这种，对应 `lsusb -t` 看到的端口树）。

写规则文件 `/etc/udev/rules.d/99-usb2can.rules`：

```udev
# 按序列号绑定 (推荐):
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", \
  ATTRS{serial}=="0001AAAA", SYMLINK+="usb2can_a", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", \
  ATTRS{serial}=="0002BBBB", SYMLINK+="usb2can_b", MODE="0666"

# 或按物理端口位置绑定 (序列号都一样时):
# SUBSYSTEM=="tty", KERNELS=="1-1.2", SYMLINK+="usb2can_a", MODE="0666"
# SUBSYSTEM=="tty", KERNELS=="1-1.4", SYMLINK+="usb2can_b", MODE="0666"
```

把里面的 `idVendor / idProduct / serial / KERNELS` 换成你实际查出来的值。生效：

```bash
sudo udevadm control --reload && sudo udevadm trigger
ls -l /dev/usb2can_a /dev/usb2can_b   # 两个都应该是软链到 /dev/ttyACMx
```

```

#### Step B — 启动双板 launch

```bash
source install/setup.bash
ros2 launch usb2can usb2can_dual.launch.py
# 或自定义符号链接 / 命名空间:
ros2 launch usb2can usb2can_dual.launch.py \
    device_a:=/dev/usb2can_a ns_a:=boardA \
    device_b:=/dev/usb2can_b ns_b:=boardB
```

启动后话题会被推到各自的 namespace 下：

| 单板（原行为） | 双板 A | 双板 B |
|----------------|--------|--------|
| `/imu/data`               | `/boardA/imu/data`            | `/boardB/imu/data` |
| `/dcu/feedback`           | `/boardA/dcu/feedback`        | `/boardB/dcu/feedback` |
| `/dcu/command`            | `/boardA/dcu/command`         | `/boardB/dcu/command` |
| `/motor/chN/cmd`          | `/boardA/motor/chN/cmd`       | `/boardB/motor/chN/cmd` |
| `/motor/chN/state`        | `/boardA/motor/chN/state`     | `/boardB/motor/chN/state` |
| `/motor/chN/motor_enable` | `/boardA/motor/chN/motor_enable` | `/boardB/motor/chN/motor_enable` |
| `/motor/chN/enable`       | `/boardA/motor/chN/enable`    | `/boardB/motor/chN/enable` |
| `/motor/chN/raw_cmd`      | `/boardA/motor/chN/raw_cmd`   | `/boardB/motor/chN/raw_cmd` |

#### Step C — 操作示例

```bash
# 给 A 板的 ch1 motor 0 / 2 周期发 MIT 命令:
ros2 topic pub -r 500 /boardA/motor/ch1/cmd usb2can/msg/MotorCommandArray "{
  channel: 1,
  motors: [
    {id: 0, position: 0.0, velocity: 0.0, kp: 30.0, kd: 1.0, torque_ff: 0.0},
    {id: 2, position: 0.0, velocity: 0.0, kp: 30.0, kd: 1.0, torque_ff: 0.0}在
  ]
}"

# 看 B 板的 ch2 反馈:
ros2 topic echo /boardB/motor/ch2/state

# motor_monitor.py 看某一块板子 (用 __ns 重映射 namespace):
ros2 run usb2can motor_monitor.py --ros-args -r __ns:=/boardA
ros2 run usb2can motor_monitor.py --ros-args -r __ns:=/boardB

# 同时看两块, 开两个终端各跑一次即可
```

#### 双板部署常见问题

**Q: 两块板子的 `dm_motors.yaml` 限幅可以分别配吗？**
A: 可以。给两块板子分别 launch，传不同的 motors_config 路径：
```bash
ros2 launch usb2can usb2can_dual.launch.py \
    motors_config_a:=$(ros2 pkg prefix usb2can)/share/usb2can/config/dm_motors_left.yaml \
    motors_config_b:=$(ros2 pkg prefix usb2can)/share/usb2can/config/dm_motors_right.yaml
```
> 当前 `usb2can_dual.launch.py` 里两块板共享同一个 `dm_motors.yaml`。如需独立配置，自己复制一份 launch 改两条 `parameters=[…]` 即可（已留出空间）。

**Q: `ros2 topic list` 看不到 `/boardA/...`？**
A: 检查：
- `ros2 node list` 里是否有 `/boardA/usb2can_node` 和 `/boardA/dm_motor_bridge`；
- launch 输出有没有 `Opened /dev/usb2can_a @ 921600 baud (low-latency)`，没有就是 udev 没生效或板子没插好；
- 确认 `ls -l /dev/usb2can_a /dev/usb2can_b` 两个软链都存在。

**Q: 我老脚本里硬编码了 `/dcu/feedback` 等绝对路径，多板会怎样？**
A: 单板默认根 namespace，老脚本继续工作；多板下要么把脚本里的话题改相对（推荐），要么按 namespace 加前缀，要么对脚本本身用 `--ros-args -r __ns:=/boardA` 启动。

**Q: 同一个进程能不能同时拉两块？**
A: 不行也不必要。`Usb2CanNode` 一个实例对应一个串口；用 launch 起两个独立节点比共进程更简单稳健（崩溃互不影响）。

**Q: 三块、四块板呢？**
A: 复制 `usb2can_dual.launch.py` 里的 `_board(...)` 调用，加上 `device_c` / `ns_c` 等参数即可，模板已经写好。

---

## 4. 测试脚本

### 4.1 双向压测

```bash
./test.sh                              # 100 Hz, cmd=0xA5
./test.sh --rate 1000 --cmd 0xA5       # 1 kHz
./test.sh --rate 2000 --cmd 0x5A
```

效果：
- 以指定频率向 `/dcu/command` 发**裸的** DcuCommand（绕过 dm bridge，直接打 STM32）
- 0.5 s 节流打印 `/imu/data` 的 acc / gyr / quat
- **每秒**打印 `/dcu/feedback` 收到帧率 + CTRL1 前 16 字节十六进制

> 这个脚本测的是 `usb2can_node` 链路本身，不经过 `dm_motor_bridge`。压测建议
> 先用它把链路打通，再上 DM bridge。

### 4.2 ROS CLI 速查
在
```bash
ros2 topic list
ros2 topic hz   /dcu/feedback          # 反馈速率
ros2 topic hz   /motor/ch1/state       # DM bridge 输出速率
ros2 topic bw   /dcu/feedback          # 字节带宽
ros2 topic info /motor/ch1/cmd -v      # QoS / 发布订阅信息
ros2 node info  /dm_motor_bridge
```

⚠️ `ros2 topic hz` 是 Python CLI，**必须在 source 过 overlay 的非-conda shell 里运行**：

```bash
conda deactivate
source /opt/ros/humble/setup.bash
source ~/USB2CAN/install/setup.bash
ros2 topic hz /motor/ch1/state
```

---

## 5. 245B 帧格式（usb2can_node 内部，了解一下即可）

下行（host → DCU）:

```
D0..D1     header (0xAA 0x55)
D2         length = 0xF2 (=242)  // payload 长度 = 总帧 - 头 - len 字段
D3..D67    CTRL1 : Cmd(1) + Payload(64)
D68..D132  CTRL2 : Cmd(1) + Payload(64)
D133..D197 CTRL3 : Cmd(1) + Payload(64)
D198       IMU CMD
D199..D242 Reserved (清零)
D243..D244 CRC-16/MODBUS (poly 0xA001, init 0xFFFF, refin/refout=true, 小端)
```
在
上行（DCU → host）:

```
D0..D1     header
D2         length
D3..D66    CTRL1 (64B, 由 dm_motor_bridge 切成 8×8B 解析)
D67..D130  CTRL2 (64B)
D131..D194 CTRL3 (64B)
D195..D234 IMU   (Acc XYZ / Gyro XYZ / Quat WXYZ, float32 LE)
D235..D242 Reserved
D243..D244 CRC-16/MODBUS
```

详细协议见 `usb2can/README.md`。

---

## 6. 低延迟实现要点

1. `ioctl(TIOCSSERIAL)` 开 `ASYNC_LOW_LATENCY`，把内核 CDC-ACM 16 ms 轮询降到 ~1 ms
2. `termios` raw 8N1，`VMIN=0 VTIME=0` 非阻塞读
3. RX 独立线程 `poll()` + `read()`，尝试 `SCHED_FIFO prio 20`
4. 帧同步：滚动缓冲按 `0xAA 0x55` 对齐 + CRC-16 校验
5. 写入互斥锁，多线程安全
6. USB 拔出 → `POLLHUP/EOF/ENODEV` 立即检测 → 2 s 自动重连
7. CRC 表用 MODBUS 标准多项式，与 STM32 端 `do_crc_table()` 完全一致

进一步压低抖动（可选）：

```bash
sudo chrt -f 80 taskset -c 2 ros2 launch usb2can usb2can_with_dm.launch.py
sudo cpupower frequency-set -g performance
echo -1 | sudo tee /sys/module/usbcore/parameters/autosuspend
```

---

## 7. 常见问题

**Q: 编译 `ModuleNotFoundError: No module named 'em'`**
A: 你在 conda 里。`conda deactivate` 或直接用 `./build.sh`（脚本自动剥离）。

**Q: `ros2 topic pub/echo/hz` 报 `The passed message type is invalid` / `The message type 'usb2can/msg/XXX' is invalid`**
A: 当前 shell **没有 source 这次 build 之后的 overlay**。新加的 msg（比如 `MotorEnableArray`）只在最新的 `install/` 里。修复：
```bash
source /home/ubuntu/USB2CAN/install/setup.bash
```
**注意**：之前 source 过、然后又重新 build 加了新消息的 shell **必须重新 source**，老的环境变量只记得老消息。
一劳永逸：
```bash
echo "source /home/ubuntu/USB2CAN/install/setup.bash" >> ~/.bashrc
```
其它常见原因：在 conda 里 `python3` 解析到 3.8 → `conda deactivate` 再 source。

**Q: `ros2 run usb2can motor_monitor.py` 报 `Package 'usb2can' not found`**
A: 同上，当前 shell 没 source overlay。`source install/setup.bash` 后再运行。
快速判断当前 shell 有没有 source：
```bash
ros2 pkg list | grep usb2can       # 有输出 = 已 source; 无 = 未 source
echo $AMENT_PREFIX_PATH | tr ':' '\n' | grep USB2CAN   # 应能看到本仓库 install 路径
```

**Q: 大量 `Serial write failed` 刷屏**
A: 串口被拔了。节点会每 2 s 自动重连，刷屏只是 warn。重新插上即可。

**Q: 启动后 `Cannot open /dev/ttyACM0`**
A: 设备没插 / 名字不对 / 权限不够。`ls /dev/ttyACM* /dev/ttyUSB*`，
   并确认在 `dialout` 组里。

**Q: 拔插 USB 后一直 `No such file or directory` 不恢复**
A: 重新枚举时设备被分到了 `/dev/ttyACM1`（不是原来的 ACM0）。
   两种解决：
   1) 临时：`./run.sh /dev/ttyACM1`
   2) 永久：用 udev 固定符号链接 `/dev/usb2can`：
      ```bash
      udevadm info -a -n /dev/ttyACM0 | head -30   # 查 idVendor/idProduct
      sudo tee /etc/udev/rules.d/99-usb2can.rules <<'EOF'
      SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", \
          SYMLINK+="usb2can", MODE="0666", GROUP="dialout"
      EOF
      sudo udevadm control --reload && sudo udevadm trigger
      ```
      `run.sh` 已经会自动优先使用 `/dev/usb2can`。

**Q: 我看到 STM32 收到的字节是 `55 AA 55 F2`，长度跑到第 4 位？**
A: STM32 端没做帧同步。需要按 `0xAA 0x55` 对齐 + 收满 245 字节 + CRC 校验。
   见 `usb2can/README.md` 末尾的最小帧同步代码。

**Q: 想换不同的 DM 电机型号？**
A: 改 `usb2can/config/dm_motors.yaml`，每颗电机一份限幅；不需要重新编译，
   只需重启 launch。详见 §9。

**Q: 发 `/motor/ch1/cmd` 后单片机端看到 `cmd=0x00`**
A: 三种可能：
   1) launch 里 `default_enable_ch1=false` 强制 cmd=0 → 改成 `true`（本仓库默认已经是）
   2) 之前发过 `/motor/ch1/enable false` 急停 → 再发一次 `true`
   3) launch 没重启 → 老进程还在跑老配置。`pgrep -fa dm_motor_bridge` 确认 PID

**Q: 想发 8 颗电机一起使能/失能，STM32 端要怎么解 `cmd=0xFF`？**
A: 协议已重定义：`cmd=0xFF` = **bitmask 全 1**（不是广播）。STM32 固件用一个
   纯 for 循环遍历 bit 即可，不要为 `0xFF` 加任何特判。详见 §10.2。

**Q: 启动 launch 后 STM32 立刻收到一帧 `cmd=0` 是否会触发失能？**
A: bridge **不会**在启动那一刻发任何 `DcuCommand`。串口字节量 0，
   直到你第一次 `ros2 topic pub`。可以用 `ros2 topic hz /dcu/command` 验证
   （启动后没有数字打印 = 没有帧发出）。

---

## 8. 二次开发钩子

- 写自己的 ROS 2 节点订阅 `/motor/chN/state`、发 `/motor/chN/cmd` 即可，**不要直接动**
  `/dcu/command` 和 `/dcu/feedback`（避免与 bridge 互相覆盖）。
- 如果完全不需要 DM 解析，只跑 `./run.sh`，自己直接拼 245B 字节流。
- 想改协议参数（不同 header / 帧长 / CRC 类型）：见 `usb2can/include/usb2can/protocol.hpp`
  和 `usb2can/src/crc16.cpp`。

---

## 9. 不同型号电机配置（per-motor 限幅 YAML）

DM 系列各型号的 `pos_max / vel_max / tor_max` 等限幅范围不同。
本桥支持**每颗电机独立配置**，写在 `usb2can/config/dm_motors.yaml` 里。

### 9.1 工作方式

- **全局默认** (`pos_max / vel_max / tor_max / kp_max / kd_max`) 适用于所有 24 颗电机。
- 任何一颗电机想覆盖默认，就在 `ch{N}.id{M}` 命名空间下列出要 override 的字段。
- 没列出的电机自动继承全局默认。
- `type` 字段（如 `"DM4310"`）**仅做日志识别用**，不影响协议解析。

### 9.2 配置示例

`@/home/ubuntu/USB2CAN/usb2can/config/dm_motors.yaml`：

```yaml
dm_motor_bridge:
  ros__parameters:
    # 全局默认
    pos_max: 12.5
    vel_max: 30.0
    tor_max: 10.0
    kp_max:  500.0
    kd_max:  5.0

    # 通道使能
    enable_ch1: true
    enable_ch2: true
    enable_ch3: true
    default_enable_ch1: true
    default_enable_ch2: true
    default_enable_ch3: true

    # ---------- per-motor 配置 ----------
    ch1:
      id0: { type: "DM4310" }                  # 用全局默认, 只是 type 标识
      id1: { type: "DM4310" }
      id2:                                     # 这颗换成大电机
        type: "DM6248"
        vel_max: 45.0
        tor_max: 20.0

    ch2:
      id0:
        type: "DM8009"
        vel_max: 45.0
        tor_max: 40.0
      id1:
        type: "DM6006"
        vel_max: 45.0
        tor_max: 12.0

    # ch3 全部用全局默认
```

### 9.3 DM 常见型号速查（出厂值，以你电机说明书为准）

| 型号 | `pos_max` | `vel_max` | `tor_max` | `kp_max` | `kd_max` |
|------|-----------|-----------|-----------|----------|----------|
| DM4310 | 12.5 | 30.0 | 10.0 | 500 | 5.0 |
| DM4340 | 12.5 | 10.0 | 28.0 | 500 | 5.0 |
| DM6006 | 12.5 | 45.0 | 12.0 | 500 | 5.0 |
| DM6248 | 12.5 | 45.0 | 20.0 | 500 | 5.0 |
| DM8009 | 12.5 | 45.0 | 40.0 | 500 | 5.0 |

### 9.4 启动后验证

启动日志会打印所有"非默认"电机的限幅：

```
[dm_motor_bridge] DM bridge ready. Global defaults: pos±12.5 vel±30.0 tor±10.0 Kp 0..500 Kd 0..5.0
[dm_motor_bridge]   ch1.id0 [DM4310]: pos±12.50 vel±30.0 tor±10.0 Kp 0..500 Kd 0..5.00
[dm_motor_bridge]   ch1.id2 [DM6248]: pos±12.50 vel±45.0 tor±20.0 Kp 0..500 Kd 0..5.00
[dm_motor_bridge]   ch2.id0 [DM8009]: pos±12.50 vel±45.0 tor±40.0 Kp 0..500 Kd 0..5.00
...
```

### 9.5 使用自定义 YAML

```bash
ros2 launch usb2can usb2can_with_dm.launch.py \
    motors_config:=/path/to/my_robot_motors.yaml
```

或直接修改 `usb2can/config/dm_motors.yaml`（不需要重新编译，只需重启 launch）。

### 9.6 为什么限幅重要？

DM MIT 协议把 `position / velocity / torque_ff` 三个 float 用**有限位数**（pos=16bit, vel=12bit, tor=12bit）线性映射到 `[-x_max, +x_max]`。**如果上位机 x_max 和电机固件 x_max 不一致**：

- 上位机给 `position=1.0 rad`，编码时除以错的 `pos_max` → 电机解码后位置变成别的数 → **走错位置**。
- 反馈解码同理：上位机 `pos_max` 不对，`/motor/chN/state` 显示的 `position` 数值与真实值不符。

所以**强烈建议**：换电机时同步在 `dm_motors.yaml` 里改对应槽位的限幅，否则会出现"指令和实际行为对不上"的怪现象。

---

## 10. STM32 端协议实现（**纯转发 + 看门狗**）

设计哲学：上位机**显式**控制每颗电机的使能/失能（通过 `/motor/chN/motor_enable` 发 `FC`/`FD`），STM32 端**不做任何边沿检测、不自动注入使能命令**。固件只做两件事：

1. **纯透明 bitmask 转发**：对每帧的每个 cmd 字节，bit `i` = 1 就把 `payload[i*8..i*8+7]` 当作一帧 8B CAN 发到 CAN ID = `i+1`。`cmd=0xFF` 是合法的 bitmask（8 位全 1），同时点对点发 8 帧。
2. **看门狗失能**：如果 `>100 ms` 没收到主机帧，自动给所有"上一帧 mask 里 bit=1"的电机发一帧 `DM_DISABLE`（`FF..FD`），防止上位机崩溃导致电机失控。

### 10.1 主机端：本仓库已经做好

- `dm_motor_bridge` 默认 `default_enable_chN=true`（通道总开关打开，cmd 字节 = slot_mask）
- 启动后未发任何 `/cmd` / `/motor_enable` 时 → 不会有任何 `DcuCommand` 发出，STM32 端串口零字节
- 急停：发 `/motor/chN/enable false` → 强制下一帧 cmd=0（仅本通道）

### 10.2 STM32 端：解帧函数实现

```c
// DM 厂家失能命令 (用作看门狗超时回退)
static const uint8_t DM_DISABLE[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD};

static uint8_t  prev_mask[3]  = {0, 0, 0};        // 仅给 watchdog 用
static uint32_t last_rx_tick  = 0;
#define DCU_TIMEOUT_MS 100

/* 处理一通道一帧 (cmd, payload[64]):
 *   cmd == 0x00            -> 该通道不转发任何 CAN
 *   cmd == 0x01..0xFF      -> 纯 bitmask, bit_i=1 -> 发 payload[i*8..i*8+7] 到 CAN ID=i+1
 *   不再有 "0xFF=广播 64B 到 ID=0" 这条路径.
 */
static void process_ctrl_channel(int ch, uint8_t cmd, const uint8_t* payload) {
    for (int i = 0; i < 8; ++i) {
        if (cmd & (1u << i)) {
            can_send(ch, /*can_id=*/i + 1, &payload[i * 8], 8);
        }
    }
    prev_mask[ch] = cmd;     // 仅供 watchdog 知道"上次有谁在被驱动"
}

/* 整帧入口 (245B 解帧 + CRC 校验通过后调用) */
void on_dcu_command(const DcuCommand* m) {
    last_rx_tick = HAL_GetTick();
    process_ctrl_channel(0, m->ctrl1.cmd, m->ctrl1.payload);
    process_ctrl_channel(1, m->ctrl2.cmd, m->ctrl2.payload);
    process_ctrl_channel(2, m->ctrl3.cmd, m->ctrl3.payload);
    /* m->imu_cmd: 如果你有 IMU 配置位, 在这里处理 */
}

/* 1 ms 系统 tick 里轮询: 主机断流 > 100 ms -> 给所有上一帧还在驱动的电机发 FD 失能 */
void dcu_watchdog_poll(void) {
    if (HAL_GetTick() - last_rx_tick > DCU_TIMEOUT_MS) {
        for (int ch = 0; ch < 3; ++ch) {
            for (int i = 0; i < 8; ++i) {
                if (prev_mask[ch] & (1u << i)) {
                    can_send(ch, i + 1, DM_DISABLE, 8);
                }
            }
            prev_mask[ch] = 0;
        }
    }
}
```

> 关键点：**没有任何 `0xFF` 特判**，`process_ctrl_channel` 是 8 行 for 循环，进什么 cmd 就发什么。

### 10.3 标准启动/停机时序

```
host:
  1. ros2 launch ...                          (STM32 串口零字节)
  2. ros2 topic pub --once .../motor_enable   {id:0,en:T},{id:2,en:T}   (cmd=0x05, payload[0..7]=FF..FC, payload[16..23]=FF..FC)
                                              -> STM32 发 2 帧 CAN(ID=1,3) -> 两颗电机进入 MIT
  3. ros2 topic pub -r 500 .../cmd            (cmd=0x05, payload = MIT 编码)
                                              -> STM32 周期发 2 帧 MIT
  4. Ctrl+C 步骤 3
                                              -> 100 ms 后 watchdog 触发, STM32 自动给 id=0,2 发 FD 失能
  (也可显式) ros2 topic pub --once .../motor_enable {id:0,en:F},{id:2,en:F}
                                              -> 立即点对点 FD, 不用等 watchdog
```

### 10.4 一键开整条总线（验证 0xFF 路径）

```bash
ros2 topic pub --once /motor/ch1/motor_enable usb2can/msg/MotorEnableArray "{
  channel: 1,
  motors: [
    {id:0,enable:true},{id:1,enable:true},{id:2,enable:true},{id:3,enable:true},
    {id:4,enable:true},{id:5,enable:true},{id:6,enable:true},{id:7,enable:true}
  ]
}"
```

STM32 端串口应看到：
- `ctrl1.cmd = 0xFF`
- `ctrl1.payload[0..7]   = FF FF FF FF FF FF FF FC`（id=0 的 DM_ENABLE）
- `ctrl1.payload[8..15]  = FF FF FF FF FF FF FF FC`（id=1 的）
- …
- `ctrl1.payload[56..63] = FF FF FF FF FF FF FF FC`（id=7 的）

固件循环 8 次，分别发出 8 帧 8B CAN 到 ID=1..8，每颗电机各自只看到自己 ID 上的一帧 `FF..FC` → 进入 MIT。**总线上未挂在本桥管理范围里的设备完全不会收到任何东西。**

---

## 11. 文件清单

```
USB2CAN/
├─ build.sh                         一键编译
├─ run.sh                           启动 usb2can_node
├─ test.sh                          启动测试脚本
├─ README.md                        本文件
└─ usb2can/                         ROS 2 包
   ├─ CMakeLists.txt
   ├─ package.xml
   ├─ README.md                     协议细节
   ├─ config/usb2can.yaml           usb2can_node 默认参数
   ├─ launch/
   │  ├─ usb2can.launch.py          只起串口桥
   │  ├─ usb2can_with_dm.launch.py  串口桥 + DM bridge (单板)
   │  └─ usb2can_dual.launch.py     双板部署 (boardA + boardB, 见 §3.9)
   ├─ msg/
   │  ├─ CanFdChannelCmd.msg        CTRL 通道 (1B Cmd + 64B Payload)
   │  ├─ DcuCommand.msg             下行: 3×CTRL + IMU
   │  ├─ DcuFeedback.msg            上行: 3×CTRL + IMU + Reserved
   │  ├─ MotorState.msg             单电机反馈
   │  ├─ MotorStateArray.msg        8 个 MotorState
   │  ├─ MotorCommand.msg           单电机命令 (MIT)
   │  └─ MotorCommandArray.msg      0..8 个 MotorCommand
   ├─ include/usb2can/
   │  ├─ crc16.hpp
   │  ├─ protocol.hpp               帧偏移 / 常量
   │  └─ serial_port.hpp            底层串口
   ├─ scripts/test_usb2can.py       Python 测试脚本
   └─ src/
      ├─ crc16.cpp
      ├─ protocol.cpp
      ├─ serial_port.cpp            低延迟串口实现 + 自动重连
      ├─ usb2can_node.cpp           串口桥节点
      └─ dm_motor_bridge.cpp        DM4310 MIT 解析层
```
