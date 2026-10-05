# DM4310 MIT 模式电机反馈（单个电机）
uint8   id          # 槽位 / 电机 ID  (0..7)
float32 position    # rad
float32 velocity    # rad/s
float32 torque      # N*m
uint8   t_mos       # °C  MOSFET 温度
uint8   t_rotor     # °C  转子温度
uint8   err         # 状态/错误位 (RX byte0 高 4 bit)
