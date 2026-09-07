#!/usr/bin/env python3
# 六关节回零并报残差
# 用法: python3 go_zero.py [速度]   默认速度 15
import sys
from arm_common import connect, show, goto

speed = int(sys.argv[1]) if len(sys.argv) > 1 else 15
mc = connect()
show(mc, "起始")
a, err = goto(mc, [0, 0, 0, 0, 0, 0], speed)
if err:
    print(">>> 回零 %s（残差目标 <1°）" % ("达标 ✔" if max(err) < 1.0 else "未达标 ✘"))
show(mc, "回零后")
