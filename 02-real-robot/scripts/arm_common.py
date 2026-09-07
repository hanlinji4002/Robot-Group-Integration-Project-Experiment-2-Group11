#!/usr/bin/env python3
# 真机小工具公共部分（pymycobot 3.6.3 + MechArm270 类；返回码 -1 不代表失败，一律以读回角度为准）
import time, sys
from pymycobot import MechArm270

def connect():
    mc = MechArm270("/dev/ttyAMA0", 1000000)
    time.sleep(0.8)
    return mc

def read6(fn, tries=5):
    for _ in range(tries):
        v = fn()
        if v and len(v) == 6:
            return v
        time.sleep(0.4)
    return None

def show(mc, tag):
    print("%s 角度: %s" % (tag, read6(mc.get_angles)))
    print("%s 坐标: %s" % (tag, read6(mc.get_coords)))

# 按这台机械臂的实际活动范围自定的限位。
# pymycobot 库内置的 MechArm270 表把 J2 限在 ±90，但手摆到桌面取物时 J2 实际到 100 多度，
# 库的表是函数内局部变量改不了，所以这里自己发指令、自己查限位。
LIMIT_MIN = [-165, -90, -180, -165, -115, -175]
LIMIT_MAX = [165, 135, 70, 165, 115, 175]

def send_angles_direct(mc, angles, speed):
    """按 pymycobot send_angles 的协议格式直接发角度，用上面自定的限位而不是库内置的表"""
    from pymycobot.common import ProtocolCode
    ints = [mc._angle2int(a) for a in angles]
    return mc._mesg(ProtocolCode.SEND_ANGLES, ints, speed, has_reply=True)

def goto(mc, target, speed, timeout=20, tol=1.5):
    """同步移动到 target（6 角），返回到位后的角度与最大残差"""
    for k, (v, lo, hi) in enumerate(zip(target, LIMIT_MIN, LIMIT_MAX)):
        if not lo <= v <= hi:
            mc.stop()
            sys.exit("!!! 目标 J%d=%.2f 超出限位 %d~%d，未发送" % (k + 1, v, lo, hi))
    print("-> 目标:", target, "速度:", speed)
    send_angles_direct(mc, target, speed)
    t0 = time.time()
    while time.time() - t0 < timeout:
        a = read6(mc.get_angles)
        if a and max(abs(x - t) for x, t in zip(a, target)) < tol:
            break
        time.sleep(0.2)
    time.sleep(0.5)
    a = read6(mc.get_angles)
    err = [round(abs(x - t), 2) for x, t in zip(a, target)] if a else None
    print("到位角度:", a, "(用时 %.1fs)" % (time.time() - t0))
    print("各关节残差:", err, "最大:", max(err) if err else None)
    return a, err

# ---------- 示教 / 抓取共用 ----------
import json, os, sys
POINTS_FILE = "/home/er/taught_points.json"

def soft(mc):
    """J1-J5 变软可示教，J6/夹爪保持刚性"""
    for j in range(1, 6):
        mc.release_servo(j)
        time.sleep(0.1)

def hold(mc):
    """恢复全部力矩"""
    for j in range(1, 7):
        mc.focus_servo(j)
        time.sleep(0.1)
    time.sleep(0.3)

def gripper(mc, state, name=""):
    """state: 0 开, 1 合"""
    r = mc.set_gripper_state(state, 50)
    time.sleep(2)
    try:
        v = mc.get_gripper_value()
    except Exception:
        v = None
    print("  夹爪%s 返回:%s 读值:%s" % (name, r, v))

def teach_point(name, tip):
    """变软 -> 手摆到 tip 描述的位置 -> 记录为 name -> 恢复力矩"""
    mc = connect()
    show(mc, "起始")
    gripper(mc, 0, "开")   # 张开夹爪再教，教出来的点才是套着物体的姿势
    if input("[1] 即将变软：J1-J5 会失去力矩往下坠，先用手扶住机械臂！  [回车继续 / q 退出] ").strip().lower() == "q":
        sys.exit(0)
    soft(mc)
    print("已变软，可以手动摆位")
    if input("[2] 把臂摆到 %s：%s  [回车记录 / q 退出] " % (name, tip)).strip().lower() == "q":
        hold(mc)
        sys.exit(0)
    a = read6(mc.get_angles)
    if not a:
        hold(mc)
        sys.exit("读不到角度，已恢复力矩并退出")
    pts = json.load(open(POINTS_FILE)) if os.path.exists(POINTS_FILE) else {}
    pts[name] = a
    json.dump(pts, open(POINTS_FILE, "w"), indent=1)
    print("  已记录 %s = %s" % (name, a))
    input("[3] 即将恢复力矩（臂会在当前位置锁住）  [回车继续] ")
    hold(mc)
    show(mc, "恢复力矩后")

def load_points(*names):
    """读 taught_points.json，缺一个就退出"""
    if not os.path.exists(POINTS_FILE):
        sys.exit("没有 %s，先跑 teach_a.py / teach_b.py 示教" % POINTS_FILE)
    pts = json.load(open(POINTS_FILE))
    missing = [k for k in names if k not in pts]
    if missing:
        sys.exit("taught_points.json 缺少 %s，先跑 teach_a.py / teach_b.py 示教" % missing)
    return pts

def lifted(pt, deg):
    """在 pt 基础上把 J2 往回收 deg 度，即抬起来（J2 正=前倾）"""
    up = list(pt)
    up[1] -= deg
    return up
