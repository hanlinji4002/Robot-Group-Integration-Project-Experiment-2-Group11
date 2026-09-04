# 01-simulation 仿真阶段

不会用的话，先看同目录的《使用说明书.md》。

```
01-simulation/
├── README.md
├── 使用说明书.md                    从零开始的操作步骤，不懂 ROS 也能照着跑
├── mecharm_grasp/                  ROS 2 功能包（包根）
│   ├── package.xml                 包的名称、版本、依赖声明
│   ├── setup.py                    安装规则：模块、数据文件、可执行入口
│   ├── setup.cfg                   可执行文件安装位置
│   ├── resource/mecharm_grasp      ament 索引标记文件
│   ├── mecharm_grasp/
│   │   ├── __init__.py             Python 包标记
│   │   ├── ros_node.py             抓取任务节点（流程 1）：数值 IK 状态机，A 点取物、B 点放置，不可达判定与日志输出
│   │   └── ros_node2.py            抓取任务节点（流程 2）：流程 1 的独立副本，改这个不影响流程 1
│   ├── model/
│   │   ├── arm_model.xacro         机械臂模型：惯量、关节限位、夹爪 mimic 关节、碰撞体
│   │   ├── theWorld.sdf            仿真世界（流程 1）：桌面、25 mm 目标方块、物理与里程计插件
│   │   └── theWorld2.sdf           仿真世界（流程 2）：方块初始位置改到 A2 点
│   ├── config/
│   │   ├── grasp.yaml              抓取参数（流程 1）：A/B 点、安全高度、工具偏移、夹爪开合角、抓取次数、日志目录
│   │   ├── grasp2.yaml             抓取参数（流程 2）：A2/B2 点，其余同上
│   │   ├── controllers.yaml        ros2_control 控制器配置：手臂、夹爪、关节状态广播（两套流程共用）
│   │   ├── GripperCalc.py          标定工具中心：读夹爪指尖与方块的真实位姿，算出虎口中心的偏差，用来定 grasp.yaml 的 tool_tip_offset
│   │   └── Gripper_touch.py        标定夹爪闭合角：手指逐步合拢，测出方块首次被碰动的角度，用来定 grasp.yaml 的 gripper_close
│   └── launch/
│       ├── sim.launch.py           一键启动流程 1：世界、机器人、控制器、状态发布、任务节点
│       └── sim2.launch.py          一键启动流程 2
└── results/                        验收数据（未上传到仓库）
    ├── README.md                   验收记录说明
    ├── results.csv                 每次抓取的结果与落点偏差
    ├── summary.txt                 汇总：成功次数、平均偏差
    ├── trajectory.csv              关节轨迹记录
    ├── errors.log                  运行中的异常日志
    └── error_tests.txt             异常测试留痕：不可达、超限位等
```

## 两套抓取流程

代码、参数、世界、启动文件各有两份，互不影响，改一套不会动到另一套。

| | 流程 1 | 流程 2 |
|---|---|---|
| 取物点 A | [0.12, 0.08] | [0.137, 0.029] |
| 放置点 B | [0.12, -0.08] | [0.052, -0.130] |
| 工作半径 | 0.144 m | 0.140 m |
| J1 转动幅度 | 67° | 80° |
| A 点偏离正前方 | 34° | 12° |
| 节点名 | grasp_task | grasp_task2 |
| 世界名 | grasp_world | grasp_world2 |
| 日志目录 | ~/mecharm_ws/grasp_logs | ~/grasp_logs2 |

流程 2 的取物点更靠前、更接近正前方但不正对，J1 转动幅度更大，工作半径略小因而逆解余量更足。两套都实测 5 次抓取全部成功。
