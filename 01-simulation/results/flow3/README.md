# 流程 3 验收记录

2026-09-07 在 Jetson（Ubuntu 22.04、ROS 2 Humble、Gazebo Fortress）完成完整 5 次抓取仿真。

## 参数与结果

| 项目 | 数值 |
|---|---|
| 取物点 A3（世界坐标，m） | [0.095, 0.095, 0.7635] |
| 放置点 B3（世界坐标，m） | [0.100, -0.090, 0.7635] |
| 安全高度 | 0.05 m |
| 世界 / 节点 | grasp_world3 / grasp_task3 |
| 物理抓取 / 仿真判定 | sim_attach=false / sim_check=true |
| 编译 | colcon build --symlink-install，1 个包成功，退出码 0 |
| 完整抓取结果 | 5/5（100%） |
| 错误日志 | errors.log 为空 |

原有成功判据为物体最终 XY 位置距离 B 点小于 0.03 m。本轮 5 次记录的落点均为 [0.1004, -0.0901, 0.7625] m，按 CSV 保留精度计算，水平偏差约 0.41 mm。高度 0.7625 m 为方块落稳后的中心高度。

## 退出阶段说明

5 次抓取完成并写出汇总后，`grasp_task3` 正常退出。随后发送 SIGINT 关闭仿真，Gazebo GUI 在 Qt/QML 析构阶段出现 segmentation fault，`ign` 最终退出码为 1；这发生在抓取任务完成之后，不影响已写出的 5/5 结果。`errors.log` 仅是任务节点的异常日志，为空并不代表整个启动日志没有错误。原始错误堆栈保留在 `launch-recorded.log` 中，本次没有修改底层 GUI 或核心抓取算法。

## 文件

- `summary.txt`：任务原始汇总。
- `results.csv`：5 次抓取成功标记与物体落点。
- `trajectory.csv`：带仿真时间戳的关节轨迹。
- `errors.log`：任务异常日志，本轮为空。
- `launch-recorded.log`：本轮从启动到退出的完整运行日志，包含启动警告和关闭信息。

本目录保存完整一轮的原始日志，不包含视频、录屏脚本、未完成轮次或构建产物。本次未执行不可达、超限位等异常测试，因此不提供 `error_tests.txt`。

## 复现命令

将 `01-simulation/mecharm_grasp` 放入 Jetson 工作区的 `src/` 后执行：

```bash
cd ~/Desktop/exp2_sim_ws
source /opt/ros/humble/setup.bash
source ~/mecharm_ws/install/setup.bash
colcon build --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=83 ROS_LOCALHOST_ONLY=1
export IGN_PARTITION=flow3_codex_sim
export DISPLAY=:1
ros2 launch mecharm_grasp sim3.launch.py gui:=true
```

本轮使用独立 ROS 域和 Gazebo 分区隔离仿真。无头运行将 `gui:=true` 改为 `gui:=false`，无需设置 DISPLAY。新日志写入 `~/grasp_logs3/`；先备份旧日志再重跑。出现“任务完成：5 次抓取成功 5 次”且任务节点退出后，用 Control+C 正常关闭 Gazebo。
