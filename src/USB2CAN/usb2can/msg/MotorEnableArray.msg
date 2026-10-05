# 一个 CTRL 通道里若干颗电机的使能/失能请求.
# bridge 一次性合并成 1 帧 DcuCommand 发给 STM32:
#   - cmd 字节 = OR over (1 << id), 让 STM32 把对应槽位作为 CAN 帧转发
#   - payload[id*8 .. id*8+7] = DM_ENABLE 或 DM_DISABLE 字节序列
# 没出现在数组里的槽位不动 (对应 bit 为 0, STM32 不转发).
#
# 安全策略 (不使用 DM 广播):
#   bridge 永远用 bitmask 逐颗发 FC/FD, 不会因为 8 颗状态一致就自动走 cmd=0xFF.
#   一次填 8 颗 -> mask 会凑齐 0xFF, 与 broadcast 协议歧义, 自动降级为 0x7F 并 warn,
#   丢掉 id=7. 想让 id=7 也响应, 再发一次 {id:7, enable:...} 即可.
#   这样保证每颗电机只接收点对点的 FC/FD, 不会误打到本桥未管理的其它电机.
std_msgs/Header   header
uint8             channel       # 1 / 2 / 3
MotorEnable[]     motors        # 0..8 个
