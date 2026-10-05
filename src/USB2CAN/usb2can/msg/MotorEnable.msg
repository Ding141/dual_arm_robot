# 单颗 DM 电机的使能 / 失能请求。
# bridge 会把它翻译成一帧 8B 原始 CAN 报文:
#   enable = true  -> FF FF FF FF FF FF FF FC  (DM 使能, 进入 MIT 模式)
#   enable = false -> FF FF FF FF FF FF FF FD  (DM 失能, 退出 MIT 模式)
# 一次性脉冲, 不会被周期 /motor/chN/cmd 覆盖.
uint8 id          # 槽位 / 电机 ID (0..7), 实际 CAN ID = id + 1
bool  enable      # true=使能, false=失能
