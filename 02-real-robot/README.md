# 02-real-robot 真机阶段

```
02-real-robot/
├── README.md
├── scripts/                                在臂内树莓派上运行的直连脚本（pymycobot 3.6.3，MechArm270 类）
│   ├── arm_common.py                       公共库：连接串口、读角度坐标、同步移动并报残差，被下面三个脚本调用
│   ├── armtest2.py                         综合测试工具：read 读状态、nod 点头判定、zero 回零、move 移动、grip 夹爪、soft 松力矩示教、hold 恢复力矩、record 存示教点、release 急停
│   ├── go_zero.py                          六关节回零并判定残差是否小于 1°
│   ├── reach_forward.py                    从零位往前探出，目标角可在命令行覆盖
│   └── demo_seq.py                         连贯演示：回零、深探、夹爪开合、J1 转 10°、回零
└── ros/                                    ROS 2 侧真机文件，跑在 Jetson 上经网络驱动臂（尚未联调）
    ├── mecharm_real_driver.py              真机驱动后端，把任务节点的轨迹指令转成臂的运动指令
    ├── real_grasp.launch.py                真机启动文件：驱动加任务节点
    └── real_grasp_params.yaml              真机参数：示教出的 A/B 点、安全高度、夹爪开合值写在这里
```

臂内 `/home/er/` 有与 scripts 同名的五个脚本，内容一致；示教点存档 `taught_points.json` 只在臂内。
