# 一个 CTRL 通道下所有电机命令。
# 数组里没出现的 id (0..7) 在该帧里全部置零（pos=vel=kp=kd=tor=0），
# 即 DM 电机 free / 力矩 0 状态。
std_msgs/Header   header
uint8             channel       # 1 / 2 / 3
MotorCommand[]    motors        # 0..8 个, 按 id 任意顺序
