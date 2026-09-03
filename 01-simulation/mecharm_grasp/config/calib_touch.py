#!/usr/bin/env python3
"""夹爪触碰角标定探针（仿真专用）。

用法：先 `ros2 launch mecharm_grasp sim.launch.py gui:=false task:=false`
再 `python3 config/calib_touch.py`（需先 source 工作区）。

原理：手臂降到取物点，手指从全开逐步合拢（0.02 rad/步），
用物体里程计检测方块首次被碰动的角度 θ_touch。
gripper_close 应取 θ_touch 再多合 0.03~0.05（轻微夹持力）。
指尖网格是爪形曲面，几何推算不可靠，实测是唯一靠谱的标定方式。
"""
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading

from control_msgs.action import FollowJointTrajectory
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from mecharm_grasp.task_node import ARM_JOINTS, GRIPPER_JOINT, solve_ik

POINT_A_LOCAL = np.array([0.12, 0.08, 0.0135 + 0.002])  # 与 config/grasp.yaml 一致
TOOL_OFF = 0.063


class Probe(Node):
    def __init__(self):
        super().__init__('calib_touch')
        cb = ReentrantCallbackGroup()
        self.arm = ActionClient(self, FollowJointTrajectory,
                                '/arm_controller/follow_joint_trajectory', callback_group=cb)
        self.hand = ActionClient(self, FollowJointTrajectory,
                                 '/hand_controller/follow_joint_trajectory', callback_group=cb)
        self.obj = None
        self.create_subscription(Odometry, '/model/target_object/odometry',
                                 self._odom, 10, callback_group=cb)
        threading.Thread(target=self.run, daemon=True).start()

    def _odom(self, m):
        p = m.pose.pose.position
        self.obj = np.array([p.x, p.y, p.z])

    def send(self, cli, joints, pos, dur):
        g = FollowJointTrajectory.Goal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = joints
        pt = JointTrajectoryPoint()
        pt.positions = [float(x) for x in pos]
        pt.time_from_start.sec = int(dur)
        pt.time_from_start.nanosec = int((dur % 1) * 1e9)
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
        while self.obj is None:
            time.sleep(0.2)
        log = self.get_logger()
        log.info('求逆解并移动到取物点...')
        above = solve_ik(POINT_A_LOCAL + [0, 0, 0.05], TOOL_OFF)
        at = solve_ik(POINT_A_LOCAL, TOOL_OFF, seed=above)
        self.send(self.hand, [GRIPPER_JOINT], [0.14], 1.0)
        self.send(self.arm, ARM_JOINTS, above, 3.0)
        self.send(self.arm, ARM_JOINTS, at, 2.0)
        # 等方块完全静止再取基准：下降扰动后的余动会被误判成触碰
        prev = self.obj.copy()
        for _ in range(20):
            time.sleep(0.5)
            if np.linalg.norm(self.obj[:2] - prev[:2]) < 0.0002:
                break
            prev = self.obj.copy()
        p0 = self.obj.copy()
        log.info(f'方块已静止，基准位置 {np.round(p0, 4)}，开始逐步合拢')
        theta = 0.14
        touch = None
        while theta > -0.74:
            theta -= 0.02
            self.send(self.hand, [GRIPPER_JOINT], [theta], 0.3)
            time.sleep(0.4)
            d = np.linalg.norm(self.obj[:2] - p0[:2])
            log.info(f'θ={theta:+.2f}  方块位移 {d*1000:.1f}mm')
            if d > 0.003:
                touch = theta
                break
        if touch is None:
            log.error('合拢到极限也没碰到方块——检查 tool_tip_offset 或 A 点')
        else:
            log.info(f'>>> 触碰角 θ_touch = {touch:.2f}，建议 gripper_close = {touch - 0.04:.2f}')
        self.send(self.hand, [GRIPPER_JOINT], [0.1], 1.0)
        rclpy.try_shutdown()


def main():
    rclpy.init()
    n = Probe()
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(n)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
