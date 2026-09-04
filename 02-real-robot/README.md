# 02-real-robot 真机阶段

Mac 通过网线直连臂内树莓派，脚本在树莓派上运行（舵机接在它的串口 /dev/ttyAMA0 上，
必须在臂里执行）。依赖 pymycobot 3.6.3，一律使用 MechArm270 类。

```
02-real-robot/
├── README.md
└── scripts/                在臂内树莓派上运行的直连脚本
    ├── arm_common.py       公共库：连接串口、读角度坐标、同步移动并报残差，被下面三个脚本调用
    ├── armtest2.py         综合测试工具：read 读状态、nod 点头判定、zero 回零、move 移动、grip 夹爪、soft 松力矩示教、hold 恢复力矩、record 存示教点、release 急停
    ├── go_zero.py          六关节回零并判定残差是否小于 1°
    ├── reach_forward.py    从零位往前探出，目标角可在命令行覆盖
    └── demo_seq.py         连贯演示：回零、深探、夹爪开合、J1 转 10°、回零
```

臂内 `/home/er/` 有与 scripts 同名的五个脚本，内容一致；示教点存档 `taught_points.json` 只在臂内。

## 两条规矩

1. 必须用 `MechArm270` 类。老 `MyCobot` 类发的运动帧会被当前固件静默丢弃，指令看似发出去了却不动。
2. 指令返回 `-1` 不代表失败，本固件的回执格式库不认。成败一律以读回角度为准。
