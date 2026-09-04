#!/usr/bin/env python3
# mechArm 270-Pi 真机测试工具 v2 —— 基于 pymycobot 3.6.3 的 MechArm270 类
# （老版 armtest.py 用的 MyCobot 类与重刷后的固件协议不合，运动指令会被固件丢弃）
# 用法:
#   python3 armtest2.py read                读角度/编码器/坐标
#   python3 armtest2.py nod [N] [D]         关节N转D度再回（默认N=1 D=+5，限幅±10，速度20）
#   python3 armtest2.py zero [SPEED]        全关节回零（默认速度15，同步等待并报残差）
#   python3 armtest2.py move j1 j2 j3 j4 j5 j6 [SPEED]  全关节移动（谨慎使用）
#   python3 armtest2.py grip open|close|0-100  夹爪
#   python3 armtest2.py soft                J1-J5变软可示教（J6/夹爪保持刚性）
#   python3 armtest2.py hold                恢复全部力矩
#   python3 armtest2.py record NAME         记录当前角度到 taught_points.json
#   python3 armtest2.py release             急停+全松（臂会变软下坠，先扶住！）
# 注意: 本固件下指令返回 -1 不代表失败，一律以读回角度为准
import sys, time, json, os
from pymycobot import MechArm270

mc = MechArm270("/dev/ttyAMA0", 1000000)
time.sleep(0.8)

def read_angles(tries=5):
    for _ in range(tries):
        a = mc.get_angles()
        if a and len(a) == 6:
            return a
        time.sleep(0.4)
    return None

def show():
    print("角度:", read_angles())
    try:
        print("编码器:", mc.get_encoders())
    except Exception:
        pass
    try:
        print("坐标:", mc.get_coords())
    except Exception:
        pass

cmd = sys.argv[1] if len(sys.argv) > 1 else "read"

if cmd == "read":
    show()

elif cmd == "nod":
    n = int(sys.argv[2]) if len(sys.argv) > 2 else 1
    d = float(sys.argv[3]) if len(sys.argv) > 3 else 5.0
    d = max(-10.0, min(10.0, d))
    a0 = read_angles()
    if not a0:
        sys.exit("读不到角度")
    print("起始:", a0)
    r = mc.send_angle(n, a0[n - 1] + d, 20)
    print("send_angle返回:", r, "(本固件-1不代表失败)")
    time.sleep(2.5)
    a1 = read_angles()
    print("转出:", a1)
    moved = bool(a1) and abs(a1[n - 1] - a0[n - 1]) > max(1.5, abs(d) * 0.4)
    mc.send_angle(n, a0[n - 1], 20)
    time.sleep(2.5)
    print("回位:", read_angles())
    print(">>> J%d %s" % (n, "动了 ✔" if moved else "没动 ✘"))

elif cmd == "zero":
    sp = int(sys.argv[2]) if len(sys.argv) > 2 else 15
    print("起始:", read_angles())
    print("-> 全关节回零，速度", sp, "(同步等待，最长20秒)")
    try:
        r = mc.sync_send_angles([0, 0, 0, 0, 0, 0], sp, timeout=20)
        print("sync返回:", r)
    except Exception as e:
        print("sync不可用(", e, ")，改用普通指令+等待")
        mc.send_angles([0, 0, 0, 0, 0, 0], sp)
        time.sleep(8)
    a1 = read_angles()
    print("回零后:", a1)
    if a1:
        err = [round(abs(x), 2) for x in a1]
        print("各关节残差:", err, "最大:", max(err))

elif cmd == "move":
    if len(sys.argv) < 8:
        sys.exit("用法: move j1 j2 j3 j4 j5 j6 [speed]")
    tgt = [float(x) for x in sys.argv[2:8]]
    sp = int(sys.argv[8]) if len(sys.argv) > 8 else 15
    print("起始:", read_angles())
    print("目标:", tgt, "速度:", sp)
    mc.send_angles(tgt, sp)
    time.sleep(4)
    print("到达:", read_angles())

elif cmd == "grip":
    arg = sys.argv[2] if len(sys.argv) > 2 else "open"
    if arg in ("open", "close"):
        r = mc.set_gripper_state(0 if arg == "open" else 1, 50)
    else:
        r = mc.set_gripper_value(int(arg), 50)
    print("返回:", r)
    time.sleep(2)
    try:
        print("夹爪值:", mc.get_gripper_value())
    except Exception:
        pass

elif cmd == "soft":
    ok = True
    for j in range(1, 6):
        try:
            mc.release_servo(j)
            time.sleep(0.1)
        except Exception as e:
            ok = False
            print("release_servo", j, "失败:", e)
    print("J1-J5已松，可手动摆位（J6/夹爪保持刚性）" if ok else "部分失败，注意扶稳")

elif cmd == "hold":
    for j in range(1, 7):
        try:
            mc.focus_servo(j)
            time.sleep(0.1)
        except Exception as e:
            print("focus_servo", j, "失败:", e)
    time.sleep(0.3)
    print("力矩已恢复，当前姿势:", read_angles())

elif cmd == "record":
    name = sys.argv[2] if len(sys.argv) > 2 else "unnamed"
    a = read_angles()
    fn = "/home/er/taught_points.json"
    data = json.load(open(fn)) if os.path.exists(fn) else {}
    data[name] = a
    json.dump(data, open(fn, "w"), indent=1)
    print("已记录", name, "=", a)

elif cmd == "release":
    mc.stop()
    mc.release_all_servos()
    print("已急停+全松，手臂是软的")

else:
    sys.exit("未知命令: " + cmd)
