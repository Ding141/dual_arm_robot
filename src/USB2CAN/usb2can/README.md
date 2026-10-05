# usb2can (ROS 2 Humble)

低延迟 USB 串口桥，用于主机与 STM32 DCU 之间通信。DCU 带有 3 路 CAN-FD 通道
（CTRL1/CTRL2/CTRL3）和 1 路 IMU。

## 帧格式（共 245 字节）

### 下行（主机 → DCU）
```
D0..D1     header (默认 0xAA 0x55)
D2         length
D3..D67    CTRL1   : Cmd(1) + Payload(64)
D68..D132  CTRL2   : Cmd(1) + Payload(64)
D133..D197 CTRL3   : Cmd(1) + Payload(64)
D198       IMU CMD
D199..D242 Reserved (清零)
D243..D244 CRC-16/MODBUS, 小端, 计算范围 D0..D242
```

### 上行（DCU → 主机）
```
D0..D1     header
D2         length
D3..D66    CTRL1   : 64 字节, 按 ID 排布 (ID i -> bytes[i*8 .. i*8+7])
D67..D130  CTRL2   : 64 字节
D131..D194 CTRL3   : 64 字节
D195..D234 IMU     : Acc(XYZ) Gyro(XYZ) Quat(WXYZ), 全部 float32, 小端
D235..D242 Reserved
D243..D244 CRC-16/MODBUS, 小端
```

## 编译

```bash
cd ~/ros2_ws/src
ln -s /home/ubuntu/USB2CAN/usb2can .
cd ~/ros2_ws
colcon build --packages-select usb2can --symlink-install
source install/setup.bash
```

## 运行

```bash
ros2 launch usb2can usb2can.launch.py device:=/dev/ttyACM0
```

或直接：

```bash
ros2 run usb2can usb2can_node --ros-args -p device:=/dev/ttyACM0 -p baudrate:=921600
```

## 话题

| 方向 | 话题 | 类型 |
|------|------|------|
| 订阅 | `/dcu/command`  | `usb2can/msg/DcuCommand`  |
| 发布 | `/dcu/feedback` | `usb2can/msg/DcuFeedback` |
| 发布 | `/imu/data`     | `sensor_msgs/msg/Imu`     |

## 参数

| 名称 | 默认值 | 说明 |
|------|--------|------|
| `device`   | `/dev/ttyACM0` | 串口设备 |
| `baudrate` | `921600`       | 波特率（USB CDC 通常忽略此设置） |
| `header0`  | `0xAA`         | 帧头第 1 字节 |
| `header1`  | `0x55`         | 帧头第 2 字节 |
| `imu_frame_id` | `imu_link` | IMU 消息 frame_id |
| `publish_feedback` | `true` | 是否同时发布完整 feedback 消息 |

## 低延迟实现要点

1. 打开串口后通过 `ioctl(TIOCSSERIAL)` 设置 `ASYNC_LOW_LATENCY`，把内核
   FTDI/CDC-ACM 默认的 16 ms 轮询周期降到 ~1 ms。
2. `termios` 配置为 raw 8N1，`VMIN=0 VTIME=0` 非阻塞读。
3. 独立 RX 线程使用 `poll()` + `read()`，并尝试设置 `SCHED_FIFO` 优先级
   以减少抖动（无 `CAP_SYS_NICE` 时会自动降级，不影响功能）。
4. 帧同步使用滚动缓冲区按头 `0xAA 0x55` 对齐 + CRC-16 校验，丢弃非法帧。
5. 写入加互斥锁，可被多线程安全调用。

## 关于 CRC-16

默认使用 CRC-16/MODBUS（poly = 0xA001 反射, init = 0xFFFF, 无最终异或，
refin/refout = true，结果以小端写入 D243..D244）。如下位机使用其它变种
（CCITT-FALSE、XMODEM 等），请修改 `src/crc16.cpp` 即可。

## 自检

可以用一个回环工具粗略验证：

```bash
ros2 topic echo /imu/data
ros2 topic pub --once /dcu/command usb2can/msg/DcuCommand "{}"
```
