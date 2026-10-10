# W3 Dual Arm Workspace

这是 `Ding141/dual_arm_robot`，负责 W3 双臂机械臂的 CAN bridge、ros2_control、真机启动和控制接口。

本仓库需要与配套的 FACTR2 仓库一起使用：

- W3 控制：`Ding141/dual_arm_robot`
- FACTR2：`Ding141/factr2`

本地推荐目录：

```text
/home/dingyj/w3_dual_arm_ws
/home/dingyj/factr2
```

源码位于 `src/`，`build/`、`install/` 和 `log/` 是本地生成目录。W3 仓库只负责发布机器人状态和接收控制命令；NEXT 的数据采集、训练和推理放在另一个仓库，通过 ROS 2 话题连接。

现有真机安装、标定和启动流程见 [src/README.md](src/README.md)，控制链路说明见 [src/docs/CONTROL_PIPELINE.md](src/docs/CONTROL_PIPELINE.md)。

当前本机右臂、ROS域74、FACTR2模型与网页的启动入口：[2026-10-10右臂启动说明](src/docs/RIGHT_ARM_NEXT_STARTUP.md)。
