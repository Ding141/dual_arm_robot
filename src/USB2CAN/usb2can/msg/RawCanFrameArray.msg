# 一个 CTRL 通道下若干原始 8B CAN 帧。
# bridge 会自动:
#   - 把每帧 8 字节复制到 payload[id*8 .. id*8+7]
#   - cmd 字节 = OR over (1<<id), 让 STM32 转发对应槽位
#   - 没出现的槽位会被覆盖为 0 (不会被转发, 因为对应 bit 为 0)
std_msgs/Header   header
uint8             channel       # 1 / 2 / 3
RawCanFrame[]     frames        # 0..7 个 (id 全选会与广播 0xFF 冲突)
