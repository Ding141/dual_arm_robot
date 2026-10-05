# Down-link command frame to the DCU (host -> STM32).
std_msgs/Header header
CanFdChannelCmd  ctrl1
CanFdChannelCmd  ctrl2
CanFdChannelCmd  ctrl3
uint8            imu_cmd
