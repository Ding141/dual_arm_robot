# DM4310 MIT 模式电机命令（单个电机）
# 物理量在节点参数 [pos_max,vel_max,tor_max,kp_max,kd_max] 范围内, 越界会被钳位。
uint8   id          # 槽位 / 电机 ID (0..7)
float32 position    # rad     (位置环目标)
float32 velocity    # rad/s   (速度前馈)
float32 kp          # 0..kp_max
float32 kd          # 0..kd_max
float32 torque_ff   # N*m    (力矩前馈)
