# 机械臂定点抓取（小组实验）

> 机器人集成小组项目 Ⅰ · 实验二 ｜ 编号：S16—S20 / E5
>
> 仿真开发 → 真机验证。**先在仿真中跑通，再上真机**；真机阶段复用仿真阶段的 ROS 2 接口，只更换设备与参数配置，不重写任务逻辑。

## 实验目标

掌握机械臂**回零、关节控制、夹爪控制和定点抓取**，把固定位置的目标物抓起并放到指定位置。本实验**不考查视觉定位**，取物点、放置点均为预先标定的固定点。

## 实验环境

| 阶段 | 环境 |
|---|---|
| 仿真 | Ubuntu 22.04 · ROS 2 Humble · Gazebo / Ignition · MoveIt 2 或 ros2_control · 机械臂 URDF 模型 |
| 真机 | Jetson Orin · mechArm 270 · 夹爪或吸盘 · 固定底座 · 急停 / 断电装置 · ≤100 g 标准目标物 |

## 阶段一：仿真开发

### 任务

1. 建立机械臂、夹爪、桌面和目标物体的仿真场景。
2. 设置固定取物点 **A**、固定放置点 **B**，以及取放过程中的**安全高度**。
3. 实现完整动作流程：

   ```
   回零 → 到达取物点上方 → 下降 → 夹取 → 抬升 → 移动到放置点 → 释放 → 回零
   ```

4. 通过 ROS 2 节点或 Action 控制机械臂，并发布机械臂状态。
5. 用**一个 Launch 文件**启动仿真、控制和状态节点。
6. 对不可达目标、无逆解、关节超限等情况进行判断并安全停止。

## 阶段二：真机验证

### 任务

1. 在桌面上标记固定取物点和固定放置区域，设置安全高度。
2. 使用与仿真阶段**相同的 ROS 2 控制接口**完成回零、抓取、放置和返回。

## 仓库结构

```
.
├── README.md
├── 01-simulation/                      仿真阶段
│   ├── README.md                       目录说明与两套抓取流程对照
│   ├── 使用说明书.md                    从零开始的操作步骤，不懂 ROS 也能照着跑
│   └── mecharm_grasp/                  ROS 2 功能包，整个目录拷进 ros2_ws/src 即可编译
│       ├── package.xml                 包的名称、版本、依赖声明
│       ├── setup.py                    安装规则：模块、数据文件、可执行入口
│       ├── setup.cfg                   可执行文件安装位置
│       ├── resource/mecharm_grasp      ament 索引标记文件
│       │
│       ├── model/                      【模型构建】
│       │   ├── arm_model.xacro         机械臂模型
│       │   ├── theWorld.sdf            仿真世界（流程 1）
│       │   └── theWorld2.sdf           仿真世界（流程 2）
│       │
│       ├── config/                     【参数设置】
│       │   ├── grasp.yaml              抓取参数（流程 1）
│       │   ├── grasp2.yaml             抓取参数（流程 2）
│       │   ├── controllers.yaml        ros2_control 控制器配置
│       │   ├── GripperCalc.py          标定工具中心
│       │   └── Gripper_touch.py        标定夹爪闭合角
│       │
│       ├── mecharm_grasp/              【ROS 任务设置】
│       │   ├── __init__.py             Python 包标记
│       │   ├── ros_node.py             抓取任务节点（流程 1）
│       │   └── ros_node2.py            抓取任务节点（流程 2）
│       │
│       └── launch/                     【仿真启动文件】
│           ├── sim.launch.py           一键启动流程 1
│           └── sim2.launch.py          一键启动流程 2
│
└── 02-real-robot/                      真机阶段
    ├── README.md                       目录说明
    ├── scripts/                        在臂内树莓派上运行的直连脚本
    │   ├── arm_common.py               公共库：连接串口、读角度坐标、同步移动并报残差
    │   ├── armtest2.py                 综合测试：读状态、点头判定、回零、移动、夹爪、示教、急停
    │   ├── go_zero.py                  六关节回零并判定残差
    │   ├── reach_forward.py            从零位往前探出
    │   └── demo_seq.py                 连贯演示：回零、深探、夹爪开合、J1 转 10°、回零
    └── ros/                            ROS 2 侧真机文件，跑在 Jetson 上（尚未联调）
        ├── mecharm_real_driver.py      真机驱动后端
        ├── real_grasp.launch.py        真机启动文件
        └── real_grasp_params.yaml      真机参数：示教出的 A/B 点等
```

验收数据（results/）未上传。

## 两套抓取流程

仿真部分做了两套互不干扰的抓取流程，代码、参数、世界、启动文件各有两份。

| | 流程 1 | 流程 2 |
|---|---|---|
| 取物点 A | [0.12, 0.08] | [0.137, 0.029] |
| 放置点 B | [0.12, -0.08] | [0.052, -0.130] |
| 工作半径 | 0.144 m | 0.140 m |
| J1 转动幅度 | 67° | 80° |

两套均实测连续抓取 5 次全部成功。启动命令把 `sim` 换成 `sim2` 即可切换。

## 代码模块化说明

仿真代码按职责拆成四个模块，互不干扰，改一处不会牵动其他部分。

**模型构建（`model/`）** 描述"机械臂长什么样、世界里有什么"。`arm_model.xacro` 是 mechArm 270 的模型，取自厂商公开的官方描述包，我们在其基础上补全了惯量、关节限位，并把夹爪的联动关节接进 ros2_control。`theWorld.sdf` 是 Gazebo 里的仿真世界，包含桌面、25 mm 的目标方块以及物理和里程计插件。

**参数设置（`config/`）** 控制机械臂"抓多高、抓多紧、抓几次"。`grasp.yaml` 里是取放点、安全高度、夹爪开合角、抓取次数等全部可调数值，改这个文件不用碰任何代码。`controllers.yaml` 是 ros2_control 的控制器配置，规定手臂和夹爪由哪个控制器驱动、控制频率多少、到位判定多严。另外两个是标定脚本：`Gripper_touch.py` 让手指逐步合拢，测出方块第一次被碰动的角度，用来确定夹爪该合到多少；`GripperCalc.py` 读取夹爪指尖和方块的真实位姿，算出虎口中心的偏差，用来确定工具中心的位置。这两个数值靠几何推算不准，必须实测，标定完写回 `grasp.yaml` 即可，平时不需要重复运行。

**ROS 任务设置（`mecharm_grasp/`）** 是 ROS 2 规定必须存在的 Python 模块目录，目录名必须与包名一致，节点代码只能放在这里，否则 ROS 找不到程序入口。`ros_node.py` 是抓取任务节点，用数值逆运动学求解关节角，以状态机的形式依次执行回零、移动、下降、夹取、抬升、放置等动作，同时负责不可达判定和日志输出。

**仿真启动文件（`launch/`）** 把上面三部分串起来。`sim.launch.py` 依次拉起 Gazebo 世界、生成机器人、启动控制器、发布机器人状态，最后启动任务节点，满足"一个 Launch 文件启动完整系统"的要求。

## 想改点什么？看这里

| 你想改的东西 | 改哪个文件 | 改什么 |
|---|---|---|
| **机械臂的动作流程**：调整动作顺序、增删动作、改变抓取逻辑或不可达的处理方式 | `mecharm_grasp/ros_node.py` | 状态机部分 |
| **抓取高度** | `config/grasp.yaml` | `safe_height` 是取放过程中抬升的高度；`point_a`、`point_b` 的第三个数是取物点和放置点本身的高度 |
| **夹爪角度** | `config/grasp.yaml` | `gripper_open` 张开角、`gripper_close` 闭合角。换目标物尺寸后建议重跑 `Gripper_touch.py` 重新标定 |
| **取物点和放置点的位置** | `config/grasp.yaml` | `point_a`、`point_b` 的前两个数。注意 mechArm 270 臂展有限，垂直抓取时可达半径约 0.13–0.15 m，放太远会无解 |
| **动作快慢** | `config/grasp.yaml` | `move_duration` 单段移动时长、`grip_duration` 夹爪动作时长 |
| **连续抓取次数** | `config/grasp.yaml` | `cycles` |
| **工具中心位置**（夹爪换了之后） | `config/grasp.yaml` | `tool_tip_offset`，用 `GripperCalc.py` 标定后填入 |
| **控制频率、到位判定的松紧** | `config/controllers.yaml` | `update_rate`、各关节的 `goal` 容差 |
| **机械臂本体或夹爪的结构** | `model/arm_model.xacro` | 连杆、关节限位、惯量、碰撞体 |
| **桌面、目标方块的位置或尺寸** | `model/theWorld.sdf` | 注意方块的初始位姿必须与 `grasp.yaml` 的 `point_a` 保持一致 |
| **启动流程**：增删启动的节点、改默认参数 | `launch/sim.launch.py` | |
