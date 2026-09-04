# 01-simulation 仿真阶段

```
01-simulation/
├── README.md
├── mecharm_grasp/                  ROS 2 功能包（包根）
│   ├── package.xml                 包的名称、版本、依赖声明
│   ├── setup.py                    安装规则：模块、数据文件、可执行入口
│   ├── setup.cfg                   可执行文件安装位置
│   ├── resource/mecharm_grasp      ament 索引标记文件
│   ├── mecharm_grasp/
│   │   ├── __init__.py             Python 包标记
│   │   └── ros_node.py             抓取任务节点：数值 IK 状态机，A 点取物、B 点放置，不可达判定与日志输出
│   ├── model/
│   │   ├── arm_model.xacro         机械臂模型：惯量、关节限位、夹爪 mimic 关节、碰撞体
│   │   └── theWorld.sdf            仿真世界：桌面、25 mm 目标方块、物理与里程计插件
│   ├── config/
│   │   ├── grasp.yaml              抓取参数：A/B 点、安全高度、工具偏移、夹爪开合角、抓取次数、日志目录
│   │   ├── controllers.yaml        ros2_control 控制器配置：手臂、夹爪、关节状态广播
│   │   ├── GripperCalc.py          标定工具中心：读夹爪指尖与方块的真实位姿，算出虎口中心的偏差，用来定 grasp.yaml 的 tool_tip_offset
│   │   └── Gripper_touch.py        标定夹爪闭合角：手指逐步合拢，测出方块首次被碰动的角度，用来定 grasp.yaml 的 gripper_close
│   └── launch/
│       └── sim.launch.py           一键启动：世界、机器人、控制器、状态发布、任务节点
└── results/                        验收数据
    ├── README.md                   验收记录说明
    ├── results.csv                 每次抓取的结果与落点偏差
    ├── summary.txt                 汇总：成功次数、平均偏差
    ├── trajectory.csv              关节轨迹记录
    ├── errors.log                  运行中的异常日志
    └── error_tests.txt             异常测试留痕：不可达、超限位等
```
