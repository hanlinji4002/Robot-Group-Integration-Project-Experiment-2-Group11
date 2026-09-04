#!/usr/bin/env python3
# 真机小工具公共部分（pymycobot 3.6.3 + MechArm270 类；返回码 -1 不代表失败，一律以读回角度为准）
import time
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

def goto(mc, target, speed, timeout=20):
    """同步移动到 target（6 角），返回到位后的角度与最大残差"""
    print("-> 目标:", target, "速度:", speed)
    try:
        mc.sync_send_angles(target, speed, timeout=timeout)
    except Exception as e:
        print("sync_send_angles 不可用(", e, ")，改用 send_angles + 等待")
        mc.send_angles(target, speed)
        time.sleep(6)
    time.sleep(0.5)
    a = read6(mc.get_angles)
    err = [round(abs(x - t), 2) for x, t in zip(a, target)] if a else None
    print("到位角度:", a)
    print("各关节残差:", err, "最大:", max(err) if err else None)
    return a, err
