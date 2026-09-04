#!/usr/bin/env python3
# 往前探：从零位把臂向前伸出（J2 正=前倾，J3 负=前臂展开）
# 用法: python3 reach_forward.py [速度]        默认速度 15
#       python3 reach_forward.py 15 j1 j2 j3 j4 j5 j6   自定义目标角
import sys
from arm_common import connect, show, goto

DEFAULT = [0, 40, -30, 0, 0, 0]

speed = int(sys.argv[1]) if len(sys.argv) > 1 else 15
target = [float(x) for x in sys.argv[2:8]] if len(sys.argv) >= 8 else DEFAULT

mc = connect()
show(mc, "起始")
goto(mc, target, speed)
show(mc, "探出后")
