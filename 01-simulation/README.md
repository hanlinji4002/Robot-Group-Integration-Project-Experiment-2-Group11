# 01-simulation 仿真阶段

这是一个**机械臂抓东西的仿真程序**。不需要真的机械臂，它在电脑里画出一个虚拟机械臂和一张桌子，
桌上有个红色小方块，程序让虚拟机械臂把方块夹起来、搬到旁边、放下、回到原位，来回做 5 次。

程序装在 **Jetson**（一个小盒子电脑）里，你用 **Mac** 连过去下命令。

---

## 一、文件结构

```
01-simulation/
├── README.md
├── mecharm_grasp/                  ROS 2 功能包（包根），整个目录拷进 ros2_ws/src 即可编译
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
│   │   ├── theWorld2.sdf           仿真世界（流程 2）：方块初始位置改到 A2 点
│   │   └── theWorld3.sdf           仿真世界（流程 3）：方块初始位置改到 A3 点
│   ├── config/
│   │   ├── grasp.yaml              抓取参数（流程 1）：A/B 点、安全高度、工具偏移、夹爪开合角、抓取次数、日志目录
│   │   ├── grasp2.yaml             抓取参数（流程 2）：A2/B2 点，其余同上
│   │   ├── grasp3.yaml             抓取参数（流程 3）：反向 A3/B3 点与独立日志目录
│   │   ├── controllers.yaml        ros2_control 控制器配置：手臂、夹爪、关节状态广播（三套流程共用）
│   │   ├── GripperCalc.py          标定工具中心：读夹爪指尖与方块的真实位姿，算出虎口中心偏差，用来定 tool_tip_offset
│   │   └── Gripper_touch.py        标定夹爪闭合角：手指逐步合拢，测出方块首次被碰动的角度，用来定 gripper_close
│   └── launch/
│       ├── sim.launch.py           一键启动流程 1：世界、机器人、控制器、状态发布、任务节点
│       ├── sim2.launch.py          一键启动流程 2
│       └── sim3.launch.py          一键启动流程 3（复用流程 1 控制算法）
└── results/                        验收数据（未上传到仓库）
    ├── README.md                   验收记录说明
    ├── results.csv                 每次抓取的结果与落点偏差
    ├── summary.txt                 汇总：成功次数、平均偏差
    ├── trajectory.csv              关节轨迹记录
    ├── errors.log                  运行中的异常日志
    └── error_tests.txt             异常测试留痕：不可达、超限位等
```

---

## 二、使用说明

假设你完全没用过 ROS。照着做就行，每一步只做一件事。

### 开始之前

需要：一台 Mac、一台 Jetson、一根连接两者的 USB 线。想**看到画面**的话，Jetson 还要接一台显示器。

⚠️ 画面出现在 Jetson 接的显示器上，**不会出现在 Mac 上**。

### 第 1 步：打开 Jetson

给 Jetson 插电，用 USB 线连上 Mac，**等一分钟**让它开机。

### 第 2 步：从 Mac 连过去

Mac 上打开「终端」，输入这行按回车：

```
ssh nvidia@192.168.55.1
```

看到 `nvidia@nvidia-desktop:~$` 就是连上了，之后你敲的命令都是在给 Jetson 下达。

### 第 3 步：告诉程序去哪找东西

```
source /opt/ros/humble/setup.bash && source ~/mecharm_ws/install/setup.bash && source ~/Desktop/exp2_sim_ws/install/setup.bash
```

相当于告诉电脑「工具箱在这三个抽屉里」，不说它找不到程序。
**每开一个新终端窗口都要重输一次。** 没有任何输出、直接回到 `$` 就是成功。

### 第 4 步：让机械臂动起来

**不看画面（推荐先用这个）**

```
ros2 launch mecharm_grasp sim.launch.py gui:=false
```

屏幕会刷很多字，这是正常的。等约 1 分半，看到 `任务完成：5 次抓取成功 5 次` 就是成功。

**看画面**

必须先输这行，**不输程序会直接崩掉**：

```
export DISPLAY=:1
```

再输：

```
ros2 launch mecharm_grasp sim.launch.py
```

然后去看 Jetson 接的那台显示器。

⚠️ 开画面很吃资源，曾把 Jetson 卡死过一次。只是验证功能就用不看画面那条。

### 第 5 步：看成绩单

```
cat ~/mecharm_ws/grasp_logs/summary.txt
```

想看每次的细节：

```
cat ~/mecharm_ws/grasp_logs/results.csv
```

每行是一次抓取，第二个数字是 `1` 代表成功。

### 第 6 步：收尾

程序还在跑就按 **Control + C** 停掉，然后清理没关干净的后台程序：

```
for p in "ig[n] gazebo" "gras[p]_task" "robot_stat[e]_publisher" "parameter_bridg[e]" "spawne[r]"; do pkill -9 -f "$p"; done
```

**每次跑完都要清一次**，不清下次会打架、报一堆奇怪的错。

---

## 三、三套抓取流程

程序里装了三套抓取动作，方块位置和日志目录不同。

| | 流程 1 | 流程 2 | 流程 3 |
|---|---|---|---|
| 取物点 A | [0.12, 0.08] | [0.137, 0.029] | [0.075, -0.115] |
| 放置点 B | [0.12, -0.08] | [0.052, -0.130] | [0.125, 0.060] |
| 工作半径 | 0.144 m | 0.140 m | 0.137–0.139 m |
| J1 转动幅度 | 67° | 80° | 82.5°（反向） |
| A 点偏离正前方 | +34° | +12° | -56.9° |
| 节点名 | grasp_task | grasp_task2 | grasp_task3 |
| 世界名 | grasp_world | grasp_world2 | grasp_world3 |
| 日志目录 | ~/mecharm_ws/grasp_logs | ~/grasp_logs2 | ~/grasp_logs3 |
| 验证状态 | Jetson 5/5 | Jetson 5/5 | IK 静态检查通过，待 Jetson 实跑 |

流程 2 的取物点更靠前、更接近正前方但不正对，J1 转动幅度更大，工作半径略小因而逆解余量更足。
流程 1/2 已实测 5 次抓取全部成功。流程 3 的全部路径点已通过同一数值 IK
静态检查，完整成功率需要在 Jetson 上运行后记录，不能用静态检查代替。

跑流程 2 就把命令里的 `sim` 换成 `sim2`：

```
ros2 launch mecharm_grasp sim2.launch.py gui:=false
```

运行反向对角搬运流程 3：

```
ros2 launch mecharm_grasp sim3.launch.py gui:=false
```

流程 3 的成绩单：

```
cat ~/grasp_logs3/summary.txt
```

看流程 2 的成绩单，路径也不一样：

```
cat ~/grasp_logs2/summary.txt
```

⚠️ **三套不能同时跑**，Gazebo 和控制器话题会互相打架。跑完一套、清理干净，再跑另一套。

---

## 四、出问题了怎么办

| 现象 | 原因和解决办法 |
|---|---|
| 连不上，提示 `No route to host` | 检查 USB 线两头插紧、Jetson 电源灯亮着；刚开机再等一分钟；还不行就拔掉 USB 重插 |
| 提示 `Package 'mecharm_grasp' not found` | 忘了第 3 步，回去重输那一长行 |
| 开了画面但显示器没反应，程序还崩了 | 忘了 `export DISPLAY=:1`，这行必须在启动命令**之前**输 |
| 卡住不动或报一堆红字 | 多半是上次没清干净。Control+C 停掉，跑第 6 步的清理命令，再重来 |
| 5 次里有失败 | 先重跑一次，偶尔失败可能是物理引擎随机误差；每次都失败就是参数被改坏了 |

---

## 五、想改点东西

改完**必须重新编译**才生效。

**改动作快慢、抓几次、夹爪松紧** → 改 `config/grasp.yaml`

| 参数 | 意思 |
|---|---|
| `cycles` | 抓几次 |
| `safe_height` | 搬运时抬多高（米） |
| `move_duration` | 每段动作用几秒，越大越慢 |
| `gripper_close` | 夹爪合多紧 |

**改方块位置** → 要**同时改两个文件**，里面的位置数字必须一致，否则机械臂会抓空：

1. `config/grasp.yaml` 的 `point_a`（去哪拿）和 `point_b`（放到哪）
2. `model/theWorld.sdf` 里方块的位置

⚠️ 机械臂手臂短，方块离底座**不能超过 15 厘米**，太远够不着。

**改完必须执行这步**：

```
cd ~/Desktop/exp2_sim_ws && source /opt/ros/humble/setup.bash && source ~/mecharm_ws/install/setup.bash && colcon build --symlink-install
```

看到 `Summary: 1 package finished` 就是编译成功，然后回到第 3 步重新开始。

---

## 六、命令速查表

| 想干嘛 | 输什么 |
|---|---|
| 连上 Jetson | `ssh nvidia@192.168.55.1` |
| 找工具箱（每次都要） | `source /opt/ros/humble/setup.bash && source ~/mecharm_ws/install/setup.bash && source ~/Desktop/exp2_sim_ws/install/setup.bash` |
| 跑流程 1，不看画面 | `ros2 launch mecharm_grasp sim.launch.py gui:=false` |
| 跑流程 2，不看画面 | `ros2 launch mecharm_grasp sim2.launch.py gui:=false` |
| 跑流程 3，不看画面 | `ros2 launch mecharm_grasp sim3.launch.py gui:=false` |
| 看画面（先输这个） | `export DISPLAY=:1` |
| 看成绩单 | `cat ~/mecharm_ws/grasp_logs/summary.txt` |
| 清理（每次跑完都要） | `for p in "ig[n] gazebo" "gras[p]_task" "robot_stat[e]_publisher" "parameter_bridg[e]" "spawne[r]"; do pkill -9 -f "$p"; done` |
| 改完代码重新编译 | `cd ~/Desktop/exp2_sim_ws && source /opt/ros/humble/setup.bash && source ~/mecharm_ws/install/setup.bash && colcon build --symlink-install` |
