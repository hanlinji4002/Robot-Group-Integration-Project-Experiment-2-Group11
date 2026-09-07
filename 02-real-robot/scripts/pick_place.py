#!/usr/bin/env python3
# 定点抓取：
#   回零 -> A 上方 -> 下降到 A -> 合爪 -> 抬回 A 上方 -> 搬运点 -> B 上方 -> 下降到 B -> 开爪 -> 抬回 B 上方 -> 回零
# 上方点：A/B 点把 J2 往回收 LIFT 度，垂直方向抬起，不用示教
# 搬运点：从 A 往零位走 RATIO 的比例，整体抬高后再横移去 B
# 用法: python3 pick_place.py [速度] [抬升角度] [回零比例]   默认 15, 25°, 0.7
# 用的是 taught_points.json 里的 at_a / at_b，先跑 teach_a.py / teach_b.py 示教
import sys, time
from arm_common import connect, show, goto, gripper, load_points, lifted

LOG_FILE = "/home/er/pick_log.txt"
ZERO = [0, 0, 0, 0, 0, 0]
MAX_ERR = 5.0      # 到位残差超过这个度数就当作没到，停下来

def toward_zero(pt, ratio):
    """从 pt 往零位走 ratio 的比例，得到抬高后的搬运姿势"""
    return [round(v * (1 - ratio), 2) for v in pt]

def log(line):
    with open(LOG_FILE, "a") as f:
        f.write(time.strftime("%Y-%m-%d %H:%M:%S ") + line + "\n")

def step(mc, name, target, speed):
    """走到一个点，残差过大就急停并退出"""
    print("\n-> %s" % name)
    a, err = goto(mc, target, speed)
    if not a or max(err) > MAX_ERR:
        mc.stop()
        print("!!! %s 没到位（残差 %s），已停止，臂保持当前姿势" % (name, err))
        log("FAIL at %s err=%s" % (name, err))
        sys.exit(1)

speed = int(sys.argv[1]) if len(sys.argv) > 1 else 15
LIFT = float(sys.argv[2]) if len(sys.argv) > 2 else 25.0
RATIO = float(sys.argv[3]) if len(sys.argv) > 3 else 0.7

pts = load_points("at_a", "at_b")
A, B = pts["at_a"], pts["at_b"]
A_UP, B_UP = lifted(A, LIFT), lifted(B, LIFT)
CARRY = toward_zero(A, RATIO)

mc = connect()
show(mc, "起始")
print("A      =", A)
print("A 上方 =", A_UP, "(J2 回收 %g°)" % LIFT)
print("搬运点 =", CARRY, "(回零比例 %g)" % RATIO)
print("B 上方 =", B_UP)
print("B      =", B)
if input("确认：目标物已放在 A 点，臂周围无人无物，一人守急停  [回车开始 / q 退出] ").strip().lower() == "q":
    sys.exit(0)

step(mc, "回零", ZERO, speed)
gripper(mc, 0, "开")
step(mc, "A 上方", A_UP, speed)
step(mc, "下降到 A", A, speed)
gripper(mc, 1, "合")
step(mc, "抬回 A 上方", A_UP, speed)
step(mc, "搬运点", CARRY, speed)
step(mc, "B 上方", B_UP, speed)
step(mc, "下降到 B", B, speed)
gripper(mc, 0, "开")
step(mc, "抬回 B 上方", B_UP, speed)
step(mc, "回零", ZERO, speed)
show(mc, "结束")
log("OK speed=%d lift=%g ratio=%g" % (speed, LIFT, RATIO))
print("\n>>> 抓取完成，结果已记到 %s" % LOG_FILE)
