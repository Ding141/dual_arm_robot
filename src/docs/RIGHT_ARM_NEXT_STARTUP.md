# 本机右臂连接及 NEXT 启动入口（2026-10-10）

本次使用 `/home/venom/dual_arm_robot` 和 `/home/venom/factr2`，`liyq-dev`，右臂CAN1、ROS_DOMAIN_ID=74、ROS_LOCALHOST_ONLY=1，无夹爪。

完整手册在配套FACTR2仓库：
- 本机：[详细启动手册](/home/venom/factr2/docs/operator_guide/RIGHT_ARM_STARTUP_20261010.md)
- 本机：[TXT版](/home/venom/factr2/docs/operator_guide/RIGHT_ARM_STARTUP_20261010.txt)
- 远程：[FACTR2 liyq-dev 手册](https://github.com/Ding141/factr2/blob/liyq-dev/docs/operator_guide/RIGHT_ARM_STARTUP_20261010.md)

控制端启动和退出前可靠支撑右臂。以下为已完成环境构建、CAN1已UP的冷启动摘要；当前进程仍在运行时不能重复启动。

每个A/B终端：
```bash
cd /home/venom/dual_arm_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=74 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

A：
```bash
ros2 launch ieir_bringup bridge.launch.py arms:=right gripper:=false
```

B：
```bash
ros2 launch ieir_bringup ui.launch.py workspace:=/home/venom/dual_arm_robot port:=8766 gripper:=false
```

打开 http://127.0.0.1:8766/，选择右臂、确认启动控制端，再切关节位置模式。应有JS broadcaster、GC、JPC三个控制器active。打开网页本身不使能；启动控制端会使能。无支撑时不能停止GC、其管理网页进程或bridge。

C健康器、D适配器、E模型8081、F末端8082命令见完整手册。默认模型副本已保存到FACTR2的`models/w3/right/c2_20261010/`。

原来的七个本机标定文件另备份于`log/backups/work_20261010/`（被Git忽略），manifest.json记录SHA。本次未修改标定；恢复到其它机器需使用该机器实际标定，不能直接套用本机零偏。

本次已保存C2五次轨迹衔接/限幅/连续制动和规划速度摩擦前馈。J6摩擦参数为试验工作值，当前模型仍有组合姿态空载偏置，不能宣称末端接触精度已验证。
