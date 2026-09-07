# 02-real-robot 真机阶段

这是**操作真机械臂**的代码。和仿真不同，这里的动作会让桌上那台真的机械臂动起来，
所以每一步都要人在旁边看着。

## 先搞清楚代码跑在哪

机械臂里装了一块**树莓派**，舵机接在它的串口上。所以：

- 代码必须在**机械臂里面**运行
- Mac 的作用是**远程登录进去下命令**，Mac 上不装也不跑这些脚本
- Mac 和机械臂之间只有一根网线，Mac 直接指挥不了舵机

臂内 `/home/er/` 有和本目录 `arm_pi/`、`script（unused）/` 同名的文件，内容一致。

---

## 一、文件结构

```
02-real-robot/
├── README.md
├── orderForReal.txt              完整操作命令单（示教 → 单次 → 多次往返 → 结果 → 急停）
├── mecharm_real/                 Jetson 上的 ROS 2 包（整个拷进 ros2_ws/src 即可编译）
│   ├── mecharm_real/
│   │   ├── real_driver.py        真机驱动：对任务节点提供与仿真 ros2_control 相同的动作/话题，底层经 TCP 调臂内 arm_server
│   │   ├── teach.py              示教：变软 → 手摆 → 记录 → 恢复力矩，换算后写进 real.yaml
│   │   ├── arm_client.py         arm_server 的 TCP 客户端（纯标准库）
│   │   └── kinematics.py         正运动学（与任务节点同参数）
│   ├── config/real.yaml          真机参数：示教好的 A/B 关节角、往返模式、速度上限、臂内服务地址
│   ├── launch/real.launch.py     一键启动：驱动 + 任务节点，跑完自动退出
│   └── package.xml / setup.py / setup.cfg / resource/
├── mecharm_grasp/                任务节点副本（与 01-simulation 同一份 ros_node.py；编译时用 01-simulation 的完整包）
│   ├── mecharm_grasp/ros_node.py
│   └── package.xml / setup.py
├── arm_pi/                       臂内树莓派 /home/er 上运行的文件
│   ├── arm_server.py             TCP 服务：把 arm_common 包成 JSON 协议给 Jetson 调用（--fake 可无臂联调）
│   ├── arm_common.py             串口连接、自定限位、直发角度、同步移动报残差、示教存点
│   └── taught_points.json        示教点原始记录
└── script（unused）/             早期纯脚本路线（不走 ROS），留作示教/点动/排错工具
    ├── armtest2.py               综合测试：读状态、点头判定、回零、移动、夹爪、示教、急停
    ├── go_zero.py / reach_forward.py / demo_seq.py
    └── teach_a.py / teach_b.py / pick_place.py
```

**两条路线：**

| | ROS 2（mecharm_real + arm_pi） | 纯脚本（script（unused）） |
|---|---|---|
| 跑在哪 | Jetson（臂内只跑 arm_server.py） | 臂内树莓派 |
| 用途 | 验收：与仿真相同的 ROS 2 接口、同一份任务节点 | 示教、点动、排错 |
| 任务逻辑 | 复用 01-simulation 的 ros_node.py，一行不改 | 每个脚本自己写 |

## 二、连接机械臂

### 先看清链路长什么样

```
Mac ──USB── 拓展坞 ──网线── 机械臂里的树莓派 ──串口── 舵机
         (网口 en10)      (用户名 er)        /dev/ttyAMA0
```

Mac 和舵机之间没有直接线路，Mac 只能登录树莓派、让树莓派去指挥舵机。

**换 WiFi、换网络环境都不影响这条链路。** 这里用的地址是网线两端自动生成的，
由树莓派网卡的物理编号算出来，不依赖路由器、不依赖 DHCP、不需要配任何东西。
所以换了网络之后，通常插上线就能直接用。

### 第 1 步：接线通电

1. Mac 插上**拓展坞**，网线一头接拓展坞的网口，另一头接机械臂上树莓派的网口
2. 机械臂电源开关拨到 **`—`**（`O` 是关，也是急停）
3. **等一分钟**让树莓派开机

⚠️ 电源砖插上后灯灭是正常的，不代表坏了。

### 第 2 步：确认网口活了

Mac 终端里输入：

```
ifconfig en10 | grep status
```

| 看到什么 | 说明 | 怎么办 |
|---|---|---|
| `status: active` | 线通了 | 进行下一步 |
| `status: inactive` | 有网口但线没通 | 检查网线两头，确认机械臂已开机满一分钟 |
| `does not exist` | 系统没看到这个网口 | 拓展坞没插好，或网口改名了，见第 5 步 |

### 第 3 步：登录

```
ssh arm
```

看到 `er@er:~$` 就是登进去了，之后敲的命令都在机械臂里执行。

**这一步成功，连接就完成了，直接跳到第三节。**

### 第 4 步：如果提示要输密码

说明 Mac 的钥匙没装进机械臂。装一次，以后就不用了。
提示密码时输小写 `elephant`，不行就试 `123`：

```
ssh-copy-id arm
```

装完再 `ssh arm`，就不会问密码了。

### 第 5 步：如果 `ssh arm` 根本连不上

按顺序排查。

**先确认网口叫什么名字。** 拓展坞插到不同的口，系统给的名字可能会变：

```
networksetup -listallhardwareports | grep -A1 AX88179B
```

输出里 `Device:` 后面那个就是网口名。如果不是 `en10`，
把 `~/.ssh/config` 里 `arm` 那段的 `%%en10` 改成新名字。

**再确认机械臂在不在线。** 这条命令让网线上的设备都报个数：

```
ping6 -c 2 -I en10 ff02::1
```

**然后看哪个是树莓派：**

```
ndp -an | grep en10
```

MAC 地址以 **`d8:3a:dd`** 开头的那一行就是机械臂，同一行前面 `fe80::` 开头的就是它的地址。

如果这个地址和 `~/.ssh/config` 里写的不一样（换了另一台机械臂就会不一样），
用文本编辑器打开 `~/.ssh/config`，把 `arm` 那段改成：

```
Host arm
    HostName fe80::这里填新地址%%en10
    User er
    ConnectTimeout 5
```

⚠️ 注意是**两个百分号** `%%`，这是 ssh 配置文件的写法，不是笔误。

改完先装钥匙再登录：

```
ssh-copy-id arm
ssh arm
```

### 第 6 步：备用方案，改用固定地址

上面都不行的话走 IPv4。机械臂网口的地址是**固定的** `10.42.0.89`，
只要把 Mac 这一端设成同一网段就能通。

打开「系统设置 → 网络 → AX88179B」，IPv4 选「手动」，填：

| 项目 | 填什么 |
|---|---|
| IP 地址 | `10.42.0.1` |
| 子网掩码 | `255.255.255.0` |
| 路由器、DNS | 留空 |

保存后用这条登录：

```
ssh er@10.42.0.89
```

⚠️ 这一步要改系统网络设置，得你自己在设置界面点。

### 连接自检清单

一步步往下走，哪一条不过就停在那儿排查。

| 检查什么 | 输什么 | 应该看到 |
|---|---|---|
| 网口活了吗 | `ifconfig en10 \| grep status` | `status: active` |
| 机械臂在线吗 | `ping6 -c 2 -I en10 ff02::1` | 有回复 |
| 哪个是它 | `ndp -an \| grep en10` | 有以 `d8:3a:dd` 开头的 MAC |
| 能登录吗 | `ssh arm` | 出现 `er@er:~$` |
| 整条路通了吗 | `ssh arm 'python3 /home/er/armtest2.py read'` | 打印六个关节的角度 |

**最后一条能出数字，说明从 Mac 到舵机整条链路都通了。**

---

## 三、操作机械臂

⚠️ 下面的命令会让真的机械臂动起来，**人必须在旁边看着**。

### 先读状态，确认能下命令

```
python3 /home/er/armtest2.py read
```

打印当前六个关节的角度、编码器值和末端坐标。**只读不动，最安全，每次开始前先跑一下。**

### 让它动

⚠️ **动之前，一只手扶住机械臂，另一只手放在电源开关上。**

**先做点头测试**，只让 J1 转 5 度再转回来，确认指令真能执行：

```
python3 /home/er/armtest2.py nod
```

看最后一行，写「动了 ✔」就是正常。
转 5 度幅度很小，**光用眼睛容易看不出来**，以打印的判定为准。

**回零**，六个关节都回零位，速度 15（范围 1–100，第一次用低速）：

python3 /home/er/go_zero.py 15
```

打印各关节残差，都小于 1° 就算到位。

**往前探**：

```
python3 /home/er/reach_forward.py 15
```

**跑一整套演示**（回零 → 深探 → 夹爪开合 → J1 转 10° → 回零）：

```
python3 /home/er/demo_seq.py 15
```

### 结束

⚠️ **断电前一定先用手扶住机械臂。** 带力矩的姿势下一断电，它会当场瘫倒砸到桌面。

正常顺序是：先 `python3 /home/er/go_zero.py 15` 回到零位，扶住，再把电源开关拨到 `O`。

---

## 四、ROS 2 路线：让真机复用仿真的任务节点

实验要求写的是「使用与仿真阶段相同的 ROS 2 控制接口」「切真机只改设备、通信和位置参数，不重写任务逻辑」。
纯脚本满足不了这条，ROS 2 路线专门为它而设。

### 结构

```
Jetson                                          机械臂里的树莓派
┌─────────────────────────────────┐             ┌──────────────────────────────┐
│ ros_node.py（与仿真同一份）       │             │ arm_server.py                │
│   ↓ FollowJointTrajectory 动作   │   TCP/JSON  │   ↓ 原样调用 arm_common.py    │
│ real_driver.py（本包）  ─────────┼────────────►│   ↓ MechArm270 类、自定限位   │
│   ↑ /joint_states 10Hz           │   :9001     │   ↓ 串口 /dev/ttyAMA0        │
└─────────────────────────────────┘             └──────────────────────────────┘
```

仿真里任务节点对着 ros2_control 的两个动作服务器和 /joint_states 说话；真机上 real_driver 提供**一模一样**的
三个接口，任务节点察觉不到区别。串口那一层不重写，直接复用 arm_common.py 里已经在真机上跑通的代码。

### 第 1 步：臂内起服务

登录机械臂后，后台启动并记下进程号：

```
setsid python3 /home/er/arm_server.py > /home/er/arm_server.log 2>&1 < /dev/null & echo $! > /home/er/arm_server.pid
tail -2 /home/er/arm_server.log
```

看到 `监听 0.0.0.0:9001` 就行。它负责把 Jetson 发来的指令转给舵机，之后可以退出这个终端。

停止它：

```
kill $(cat /home/er/arm_server.pid)
```

⚠️ 服务运行期间它占着串口，**不能同时跑 armtest2.py、go_zero.py 这些直连脚本**，会互相抢串口。
要用直连脚本先把服务停掉，用完再起。

### 第 2 步：Jetson 编译

把 `mecharm_real` 拷进和仿真包同一个工作区（任务节点在仿真包里）。
Jetson 上这个工作区是 `~/Desktop/exp2_sim_ws`，里面已有 `mecharm_grasp` 和 `mecharm_real`，
`~/mecharm_ws` 只作底层依赖（官方描述包、gz_ros2_control）：

```
cp -r 02-real-robot/mecharm_real ~/Desktop/exp2_sim_ws/src/
cd ~/Desktop/exp2_sim_ws && source /opt/ros/humble/setup.bash && source ~/mecharm_ws/install/setup.bash && colcon build --symlink-install
source install/setup.bash
```

每开一个新终端都要先加载三层环境：

```
source /opt/ros/humble/setup.bash && source ~/mecharm_ws/install/setup.bash && source ~/Desktop/exp2_sim_ws/install/setup.bash
```

Jetson 要能连到臂内服务：把臂的网线接到 Jetson 网口（`arm-share` 共享模式下臂是 `10.42.0.89`），
或改 `config/real.yaml` 的 `arm_host`。

### 第 3 步：示教 A、B 点

```
ros2 run mecharm_real teach a      # 张开夹爪 -> 变软 -> 手把臂摆到套住目标物 -> 回车记录 -> 恢复力矩
ros2 run mecharm_real teach b      # 同上，摆到放置位置
ros2 run mecharm_real teach show   # 看两个点及其换算出的坐标、工具轴倾角、工作半径
ros2 run mecharm_real teach apply  # 写进 real.yaml 的 point_a / point_b
```

⚠️ 变软那一步 J1-J5 会失去力矩往下坠，回车前先用手扶住机械臂。

`apply` 默认打开**示教回放模式**（`use_taught_joints: true`）：任务节点直接回放你摆的关节角作为取物/放置姿势，
上方点由 J2 往回收 `lift_deg`（默认 25°）得到，不做逆解，所以不受"工具轴要竖直"的限制，
你的取物点离底座多远、爪子斜多少都照样跑——这和 `pick_place.py` 的做法一致。
`show` 里的倾角/半径告警只对逆解模式有意义；想用逆解模式就把 `use_taught_joints` 改回 false
（那样取物点要在底座 0.15 m 以内、夹爪接近竖直向下）。

### 第 4 步：跑

先单次低速：

```
ros2 launch mecharm_real real.launch.py cycles:=1
```

没问题再跑 5 次：

```
ros2 launch mecharm_real real.launch.py
```

结果在 `~/grasp_logs_real/`，格式和仿真完全一样（results.csv / summary.txt / trajectory.csv / errors.log）。
真机没有物体位姿真值，每次全部步骤走完即计成功，落点由现场人工核对并记录。

**往返模式（默认开）**：`real.yaml` 里 `alternate_direction: true`，奇数轮 A→B、偶数轮 B→A，
物体放下后正好在下一轮的取物点，5 轮中间不用人手放回。`cycles: 5` 就是 5 次抓取、每次都计入 results.csv，
说明栏会标 `A→B` / `B→A`。某轮失败时下一轮仍按顺序走，物体在哪由现场人放回对应点。
想恢复"每轮都从 A 取、放到 B"就把它改成 false。

软件急停（另开终端，硬件急停优先）：

```
ros2 topic pub --once /soft_stop std_msgs/msg/Bool "data: true"
```

### 没有臂也能联调

`arm_server.py --fake` 会起一个假臂（角度按速度线性逼近），Jetson 上照常跑 launch，用来验证整条 ROS 链路：

```
python3 ~/Desktop/exp2_real_scripts/arm_server.py --fake &
ros2 launch mecharm_real real.launch.py host:=127.0.0.1 cycles:=1
```

2026-09-07 已在 Jetson 上实测：假臂一轮 11 步全部经驱动执行，日志正常。

### 和验收条款的对应

| 条款 | 怎么满足 |
|---|---|
| 相同的 ROS 2 控制接口 | real_driver 提供与 ros2_control 同名同类型的两个动作服务器和 /joint_states |
| 只改设备、通信、位置参数，不重写任务逻辑 | 任务节点 ros_node.py 原样复用；改动全在 real.yaml |
| 初次低速 | `max_speed: 20`，速度按「角度/时长」换算再封顶 |
| 失败 / 不可达 / 通信异常时停止或回安全位 | 逆解失败拒绝启动；到位残差超 3° 或通信异常判失败并回零；/soft_stop 急停 |
| 保存轨迹、结果、错误日志 | 与仿真同一套日志代码 |

## 五、armtest2.py 的九个命令

```
python3 /home/er/armtest2.py <命令>
```

| 命令 | 作用 |
|---|---|
| `read` | 读角度、编码器、坐标。只读不动，最安全 |
| `nod [关节号] [角度]` | 让某个关节转一点再转回来，判断指令是否生效。默认 J1 转 5° |
| `zero [速度]` | 六关节回零并报残差 |
| `move j1 j2 j3 j4 j5 j6 [速度]` | 六个关节移动到指定角度。**谨慎使用**，先想清楚会撞到什么 |
| `grip open` / `close` / `0-100` | 夹爪张开、闭合，或指定开合量 |
| `soft` | 松开 J1–J5 的力矩，手可以搬动机械臂，用来示教 |
| `hold` | 恢复力矩，锁住当前姿势 |
| `record 名字` | 把当前角度存进 `/home/er/taught_points.json` |
| `release` | 急停，全部关节松开 |

⚠️ `soft` 和 `release` 会让机械臂**变软下坠**，执行前必须先用手托住前臂。
断电重启后舵机默认全部抱闸，示教前要先 `soft`。

**示教的完整流程**：`soft` → 用手把机械臂摆到想要的位置 → `record 点位名` → 摆下一个位置再 `record` → 全部记完后 `hold` 恢复力矩。

---

## 六、两条铁规矩

1. **必须用 `MechArm270` 类，pymycobot 版本 3.6.3。**
   老的 `MyCobot` 类发出的运动帧会被当前固件静默丢弃——指令看起来发出去了，机械臂却一动不动，
   而且读状态、开关灯这类命令还是正常的，非常有迷惑性。

2. **指令返回 `-1` 不代表失败。**
   当前固件的回执格式库不认识，返回值没有参考价值。
   **成败一律以读回的角度为准**，这也是所有脚本的判定方式。

---

## 七、出问题了怎么办

| 现象 | 原因和解决办法 |
|---|---|
| `ssh arm` 提示 `No route to host` | 网线没插好，或树莓派还没开机完。按第二节第 2 步逐条排查 |
| 提示 `Could not resolve hostname arm` | `~/.ssh/config` 里没有 `arm` 这段，见第二节第 5 步 |
| 读得到角度但机械臂不动 | 多半是用了老的 `MyCobot` 类。确认脚本里是 `from pymycobot import MechArm270` |
| 机械臂突然瘫下去 | 执行了 `soft` 或 `release`，力矩被松开了。扶住它，运行 `armtest2.py hold` 恢复 |
| 树莓派连不上也 ping 不通 | 断电重新上电。之前出现过一次无故死机，重启后正常 |
