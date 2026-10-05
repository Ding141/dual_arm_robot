# 第 2 部分：插值后目标只读接口验收

结论 **PASS**，范围 **PASS_OFFLINE**，2026-10-05（Asia/Shanghai）。基线 W3 `00740ac`，FACTR2 第 1 部分 `bf18abb`，均 dingyj-dev，开始时无未提交变更。本报告所在 commit 是 W3 阶段 2 交付。完整日志及 SHA256 清单另存 FACTR2 docs/acceptance/2_evidence.json。

实现：configure 创建 ~/command_state（默认 /joint_position_controller/command_state），sensor_msgs/JointState，position/velocity 为本次 update 写入接口的同一值，effort=[]，stamp=time，name 保持原顺序。SensorDataQoS BEST_EFFORT/VOLATILE/KEEP_LAST depth=5。inactive 不制造新目标；旧排队帧用源 stamp 过期。未改插值/gain/标定/CAN/effort 所有权。

| 验收条目 | 命令/实验 | 证据 | 结论 |
|---|---|---|---|
| CMD-01 | 真 controller + double mock 接口 + ROS subscriber，无轨迹 | CommandStateTest.CMD01HoldAndCMD10StampFieldsQoS，激活测量姿态、速度0 | PASS |
| CMD-02 | 0→0.2，1s，t=.5s | CMD02LinearAndCMD05EndHold，q=.1、v=.2 | PASS |
| CMD-03 | 首点延迟1s，t=.25/.5/.75 | CMD03FrozenPreRollAndCMD04Boundary，q=.05/.1/.15，v=.2 | PASS |
| CMD-04 | t=0、未来header、未来开始t=0 | 同上；首点位置、v=0，与现有接口一致 | PASS |
| CMD-05 | t=1s/1.1s | CMD02LinearAndCMD05EndHold，末点保持、v=0，stamp前进 | PASS |
| CMD-06 | 乱序部分关节、替换轨迹 | CMD06PartialReorderedAndReplacement，未约束关节保持旧目标 | PASS |
| CMD-07 | 14和7关节 | CMD07DualAndSingle，原name顺序、数组长度、全部接口一致 | PASS |
| CMD-08 | 停用、inactive新轨迹、测量变化、重新激活 | CMD08DeactivateAndReactivate，停发、清旧轨迹、保持新测量 | PASS |
| CMD-09 | 持有RT锁、慢订阅者不spin、无订阅者 | CMD09LockContentionAndSlowOrAbsentSubscriber，锁失败update正常且不发该stamp；1000 update不等待subscriber | PASS |
| CMD-10 | 所有 step 消息与接口比对 | stamp纳秒精确、finite、effort空，q/v容差1e-9；真实 endpoint QoS | PASS |
| 包构建 | bash src/scripts/build_workspace.sh "$PWD" --packages-up-to ieir_controllers（0） | log/acceptance2/build_final.log，3相关包通过 | PASS |
| 完整测试 | 独立 W3 shell colcon test --base-paths src --packages-select ieir_controllers --event-handlers console_direct+；colcon test-result --verbose（0） | log/acceptance2/test_final.log，7个gtest + 144个原Python回归通过；colcon汇总155含包装用例 | PASS |
| RT审查 | 检查 configure/update diff | 新路径分配/resize/name填充仅configure；update一次trylock、stamp和索引写、unlockAndPublish；普通publish只在RealtimePublisher worker | PASS |

测试环境：Humble、系统Python3.10、W3 overlay、无NEXT venv；domain74，rmw_fastrtps_cpp，localhost-only=1，mock使用 FACTR2 config/w3/dds_loopback.xml。未启动 bridge、controller_manager、硬件插件或 CAN。

首次 pytest 未 source W3 overlay，缺少 w3_robot_bridge，改用正确 shell 后基线144通过。受限构建准备阶段停留，中止后在允许子进程的执行环境构建，产物仍限项目。初次编译修正Humble QoS枚举；初次未来起点测试修正手工update的ROS clock类型，最终重跑无失败。

边界记录：t_elapsed<=0（含未来轨迹）返回首点且v=0，随后正t pre-roll从pickup冻结姿态插值。现有行为可能带来位置跳变，本任务保留并如实观测；若需更改应单独控制任务。

`test/command_state_mock.cpp` 为第3部分真实controller联调提供300Hz mock joint_states/command_state，不加载硬件。运行 build/ieir_controllers/command_state_mock left 65。真机时序观察留第7部分；本验收不证明CAN实际接收时间。
