#!/usr/bin/env python3
# 连贯演示：回零 -> 往前深探 -> 夹爪开/合/开 -> J1 转 +10° 再回 -> 回零
# 用法: python3 demo_seq.py [速度]   默认 15
import sys, time
from arm_common import connect, show, goto, read6

REACH = [0, 55, -35, 0, 0, 0]   # 比 reach_forward 更低更远
J1_TURN = 10

speed = int(sys.argv[1]) if len(sys.argv) > 1 else 15
mc = connect()
show(mc, "起始")

print("\n[1] 回零"); goto(mc, [0]*6, speed)
print("\n[2] 深探"); goto(mc, REACH, speed); show(mc, "探出后")

print("\n[3] 夹爪 开 -> 合 -> 开")
for st, name in ((0, "开"), (1, "合"), (0, "开")):
    r = mc.set_gripper_state(st, 50)
    time.sleep(2)
    try:
        v = mc.get_gripper_value()
    except Exception:
        v = None
    print("  夹爪%s 返回:%s 读值:%s" % (name, r, v))

print("\n[4] J1 转 +%d° 再回" % J1_TURN)
t = list(REACH); t[0] = J1_TURN
goto(mc, t, speed)
goto(mc, REACH, speed)

print("\n[5] 回零"); a, err = goto(mc, [0]*6, speed)
if err:
    print(">>> 回零 %s" % ("达标 ✔" if max(err) < 1.0 else "未达标 ✘"))
show(mc, "结束")
