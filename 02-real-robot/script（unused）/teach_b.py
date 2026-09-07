#!/usr/bin/env python3
# 【讲解】纯脚本示教放置点 B。ROS 路线用 ros2 run mecharm_real teach b 代替。
# 示教放置点 B：变软 -> 手摆 -> 记录 at_b -> 恢复力矩
# 用法: python3 teach_b.py
from arm_common import teach_point
teach_point("at_b", "放置点 B（放下物体的位置）")
print("\n>>> 下一步: python3 go_zero.py，然后 python3 pick_place.py")
