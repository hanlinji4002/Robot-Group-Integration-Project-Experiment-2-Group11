#!/usr/bin/env python3
# 示教取物点 A：变软 -> 手摆 -> 记录 at_a -> 恢复力矩
# 用法: python3 teach_a.py
from arm_common import teach_point
teach_point("at_a", "取物点 A（夹爪张开套住目标物）")
print("\n>>> 下一步: python3 teach_b.py（已教过 B 就直接 go_zero.py）")
