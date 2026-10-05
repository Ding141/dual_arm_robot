# 一个 CTRL 通道(8 槽位)的全部反馈
std_msgs/Header header
uint8           channel         # 1 / 2 / 3
MotorState[8]   motors          # 顺序 = 槽位 0..7
