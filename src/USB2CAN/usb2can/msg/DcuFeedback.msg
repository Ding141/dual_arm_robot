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
