#!/usr/bin/env python3
"""示教工具（Jetson 侧，纯标准库，不需要 rclpy）。

通过臂内 arm_server.py 完成「变软 -> 手摆 -> 记录 -> 恢复力矩」，
再用与任务节点相同的正运动学把示教关节角换算成 point_a / point_b 写进 real.yaml。
这样任务节点（ros_node.py）不用改，只是参数变了——正是验收要求的做法。

用法：
  ros2 run mecharm_real teach a            # 示教取物点 A（夹爪张开套住目标物）
  ros2 run mecharm_real teach b            # 示教放置点 B
  ros2 run mecharm_real teach show         # 看已示教的点及其换算结果
  ros2 run mecharm_real teach apply        # 把 A/B 换算后写进 real.yaml
  可加 --host 10.42.0.89 --port 9001 --yaml /path/real.yaml --json taught_points.json（离线换算）
"""
import argparse
import json
import math
import os
import re
import sys

# 终端不是 UTF-8（如 nohup / systemd 下 LANG=C）时中文日志也不能把进程打死
def _utf8_stdout():
    import io, sys
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except AttributeError:  # Python < 3.7
        sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace", line_buffering=True)
_utf8_stdout()

from mecharm_real.arm_client import ArmClient, ArmError
from mecharm_real.kinematics import tip_pos, tilt_deg, reach, MAX_REACH, deg_to_rad

# 任务节点 plan_waypoints 在 point_a/point_b 上叠加的高度（at_a = A + 0.002，at_b = B + 0.008），
# 示教教的是 at_a / at_b 本身，换算回 point 时要减掉
AT_A_LIFT, AT_B_LIFT = 0.002, 0.008
MAX_TILT = 15.0   # 任务节点 IK 允许的工具轴倾角


def default_yaml():
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('mecharm_real'), 'config', 'real.yaml')
    except Exception:
        return None


def read_yaml_scalar(text, key, default):
    m = re.search(r'^\s*%s:\s*([-\d.]+)' % re.escape(key), text, re.M)
    return float(m.group(1)) if m else default


def convert(name, angles_deg, tool_offset, lift):
    q = deg_to_rad(angles_deg)
    p, axis = tip_pos(q, tool_offset)
    point = [round(p[0], 4), round(p[1], 4), round(p[2] - lift, 4)]
    return point, tilt_deg(axis), reach(p)


def report(name, angles, tool_offset, lift):
    point, tilt, r = convert(name, angles, tool_offset, lift)
    flags = []
    if tilt > MAX_TILT:
        flags.append('工具轴倾角 %.1f° > %d°，任务节点逆解可能不认' % (tilt, MAX_TILT))
    if r > MAX_REACH:
        flags.append('超出工作半径 %.3f > %.2f' % (r, MAX_REACH))
    print('  %-5s 关节角 %s' % (name, angles))
    print('        -> point %s  倾角 %.1f°  半径 %.3f m  %s' % (point, tilt, r, '⚠ ' + '；'.join(flags) if flags else '✔'))
    return point, flags


def teach_one(cli, name, tip):
    print('[0] 张开夹爪，先套住目标物再教')
    cli.gripper(0)
    if input('[1] 即将变软：J1-J5 失去力矩会往下坠，先用手扶住机械臂！ [回车继续 / q 退出] ').strip().lower() == 'q':
        return None
    cli.soft()
    print('已变软，可以手动摆位')
    if input('[2] 把臂摆到 %s：%s  [回车记录 / q 退出] ' % (name, tip)).strip().lower() == 'q':
        cli.hold()
        return None
    resp = cli.record(name)
    print('  已记录 %s = %s' % (name, resp['angles']))
    input('[3] 即将恢复力矩（臂会在当前位置锁住） [回车继续] ')
    cli.hold()
    return resp['angles']


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('what', choices=['a', 'b', 'show', 'apply'])
    ap.add_argument('--host', default='10.42.0.89')
    ap.add_argument('--port', type=int, default=9001)
    ap.add_argument('--yaml', default=default_yaml())
    ap.add_argument('--json', default=None, help='离线：直接读 taught_points.json，不连臂')
    args = ap.parse_args()

    if not args.yaml or not os.path.exists(args.yaml):
        sys.exit('找不到 real.yaml，请用 --yaml 指定')
    ytext = open(args.yaml, encoding='utf-8').read()
    tool_offset = read_yaml_scalar(ytext, 'tool_tip_offset', 0.063)

    if args.json:
        pts = json.load(open(args.json))
        cli = None
    else:
        cli = ArmClient(args.host, args.port)
        try:
            cli.ping()
        except ArmError as e:
            sys.exit('连不上臂内服务 %s:%d（%s）。先在臂内跑 python3 /home/er/arm_server.py' % (args.host, args.port, e))
        pts = None

    if args.what in ('a', 'b'):
        if cli is None:
            sys.exit('示教需要连臂，不能用 --json')
        name, tip = ('at_a', '取物点 A（夹爪张开套住目标物）') if args.what == 'a' else ('at_b', '放置点 B（放下物体的位置）')
        angles = teach_one(cli, name, tip)
        if angles is None:
            print('已取消'); return
        print('\n换算结果（tool_tip_offset=%.3f）:' % tool_offset)
        report(name, angles, tool_offset, AT_A_LIFT if args.what == 'a' else AT_B_LIFT)
        print('\n>>> 下一步: %s' % ('ros2 run mecharm_real teach b' if args.what == 'a' else 'ros2 run mecharm_real teach apply'))
        return

    pts = pts if pts is not None else cli.points()
    missing = [k for k in ('at_a', 'at_b') if k not in pts]
    if missing:
        sys.exit('示教点缺少 %s，先跑 teach a / teach b' % missing)
    print('已示教的点（tool_tip_offset=%.3f）:' % tool_offset)
    pa, fa = report('at_a', pts['at_a'], tool_offset, AT_A_LIFT)
    pb, fb = report('at_b', pts['at_b'], tool_offset, AT_B_LIFT)
    if args.what == 'show':
        return
    if fa or fb:
        if input('存在告警，仍要写入 real.yaml 吗？ [y/N] ').strip().lower() != 'y':
            print('未写入'); return
    new = re.sub(r'^(\s*point_a:\s*)\[.*\]', lambda m: m.group(1) + str(pa), ytext, count=1, flags=re.M)
    new = re.sub(r'^(\s*point_b:\s*)\[.*\]', lambda m: m.group(1) + str(pb), new, count=1, flags=re.M)
    if new == ytext:
        sys.exit('real.yaml 里没找到 point_a / point_b 行，未修改')
    open(args.yaml, 'w', encoding='utf-8').write(new)
    print('\n已写入 %s\n  point_a: %s\n  point_b: %s' % (args.yaml, pa, pb))
    print('>>> 下一步: ros2 launch mecharm_real real.launch.py cycles:=1')


if __name__ == '__main__':
    main()
