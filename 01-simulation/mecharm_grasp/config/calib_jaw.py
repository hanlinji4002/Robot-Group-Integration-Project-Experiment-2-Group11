#!/usr/bin/env python3
"""实测虎口中心偏差（仿真标定工具）。

移动到取物点位姿后，从 Gazebo 读左右指尖链节和方块的真实位姿，
输出虎口中心相对方块中心的偏差（世界系 + 夹爪系），
用于修正任务节点的工具中心模型。
"""
import re
import subprocess
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from mecharm_grasp.task_node import (ARM_JOINTS, GRIPPER_JOINT,
                                           solve_ik, align_jaw, fk)

BASE = np.array([0.0, 0.0, 0.75])
POINT_A_LOCAL = np.array([0.12, 0.08, 0.0135 + 0.002])
TOOL_OFF = 0.060


def get_poses():
    out = subprocess.run(
        ['ign', 'topic', '-e', '-n', '1', '-t', '/world/grasp_world/pose/info'],
        capture_output=True, text=True, timeout=5).stdout
    poses = {}
    for m in re.finditer(
            r'name:\s*"([^"]+)"[^}]*?position\s*{\s*(?:x:\s*([-\d.e+]+))?\s*(?:y:\s*([-\d.e+]+))?\s*(?:z:\s*([-\d.e+]+))?\s*}',
            out):
        name = m.group(1)
        poses[name] = np.array([float(m.group(i) or 0) for i in (2, 3, 4)])
    return poses


class Measure(Node):
    def __init__(self):
        super().__init__('calib_jaw')
        cb = ReentrantCallbackGroup()
        self.arm = ActionClient(self, FollowJointTrajectory,
                                '/arm_controller/follow_joint_trajectory', callback_group=cb)
        self.hand = ActionClient(self, FollowJointTrajectory,
                                 '/hand_controller/follow_joint_trajectory', callback_group=cb)
        threading.Thread(target=self.run, daemon=True).start()

    def send(self, cli, joints, pos, dur):
        g = FollowJointTrajectory.Goal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = joints
        pt = JointTrajectoryPoint()
        pt.positions = [float(x) for x in pos]
        pt.time_from_start.sec = int(dur)
        g.trajectory.points = [pt]
        f = cli.send_goal_async(g)
        while not f.done():
            time.sleep(0.02)
        r = f.result().get_result_async()
        while not r.done():
            time.sleep(0.02)

    def run(self):
        self.arm.wait_for_server()
        self.hand.wait_for_server()
        log = self.get_logger()
        above = align_jaw(solve_ik(POINT_A_LOCAL + [0, 0, 0.05], TOOL_OFF))
        at = align_jaw(solve_ik(POINT_A_LOCAL, TOOL_OFF, seed=above))
        self.send(self.hand, [GRIPPER_JOINT], [0.14], 1.0)
        self.send(self.arm, ARM_JOINTS, above, 3.0)
        self.send(self.arm, ARM_JOINTS, at, 2.0)
        time.sleep(1.0)
        p = get_poses()
        need = ['left_finger', 'right_finger', 'gripper_base', 'link6', 'target_object']
        missing = [n for n in need if n not in p]
        if missing:
            log.error(f'pose/info 缺少: {missing}; 实际键: {sorted(p.keys())[:20]}')
        else:
            # mecharm 的链节位姿相对模型原点(0,0,0.75)，方块是独立模型为世界系
            l1 = p['left_finger'] + BASE
            r1 = p['right_finger'] + BASE
            gb = p['gripper_base'] + BASE
            cube = p['target_object']
            jaw_mid = (l1 + r1) / 2
            T6, Tg = fk(at)
            tip_model = BASE + Tg[:3, 3] + T6[:3, 2] * TOOL_OFF
            log.info(f'左指尖链节 {np.round(l1,4)}')
            log.info(f'右指尖链节 {np.round(r1,4)}')
            log.info(f'两链节中点 {np.round(jaw_mid,4)}')
            log.info(f'gripper_base {np.round(gb,4)}')
            log.info(f'方块中心   {np.round(cube,4)}')
            log.info(f'模型预测tip {np.round(tip_model,4)}')
            log.info(f'>>> 链节中点-方块 偏差 {np.round((jaw_mid-cube)*1000,1)} mm')
            log.info(f'>>> 模型tip-方块  偏差 {np.round((tip_model-cube)*1000,1)} mm')
        rclpy.try_shutdown()


def main():
    rclpy.init()
    n = Measure()
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(n)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
