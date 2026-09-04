#!/usr/bin/env python3
"""抓取流程 2（独立副本）。与流程 1 完全不共享代码，改这里不影响 ros_node.py。

定点抓取任务节点（状态机）。

流程：回零 → A点上方 → 下降 → 夹取 → 抬升 → B点上方 → 下降 → 释放 → 回零。
A/B 点、安全高度等全部来自参数文件；控制接口只用 ros2_control 的
FollowJointTrajectory 动作，仿真与真机共用本节点，切真机只换硬件后端。

关键设计：
- 逆解：自带数值 IK（阻尼最小二乘 + 数值雅可比），约束"指尖到点 + 工具竖直
  向下"。IK 不收敛 = 目标不可达，任务拒绝启动并输出错误（验收要求）。
- 限位保护：每个下发的关节目标都先经限位校验，超限即报错停止。
- 仿真吸附：Gazebo 里夹爪与物体的接触物理很难稳定，通用做法是夹取判定
  成功后将物体"吸附"跟随工具运动（10Hz 调 Gazebo set_pose 服务）。
  真机上 sim_attach=false，该逻辑整体停用，真实夹爪自然接管。
- 日志：轨迹 CSV、逐次结果、错误日志写入 log_dir（验收要求）。
"""
import math
import os
import subprocess
import threading
import time
from datetime import datetime

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import String

ARM_JOINTS = [
    'joint1_to_base', 'joint2_to_joint1', 'joint3_to_joint2',
    'joint4_to_joint3', 'joint5_to_joint4', 'joint6_to_joint5',
]
GRIPPER_JOINT = 'gripper_controller'

# 关节限位，与 URDF 一致（rad）
JOINT_LIMITS = [
    (-2.792527, 2.792527), (-1.3089, 2.0943), (-3.0543, 1.1344),
    (-2.7052, 2.7052), (-2.0071, 2.0071), (-3.14, 3.14),
]

# ---------- 运动学：链参数照抄 URDF 的 joint origin ----------

def rpy_mat(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])


def make_T(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = rpy_mat(*rpy)
    T[:3, 3] = xyz
    return T


# 各关节前置静态变换（parent→child origin），之后绕子坐标系 z 轴转 q
CHAIN = [
    make_T([0, 0, 0.1], [0, 0, 0]),
    make_T([0, 0, 0.038], [-1.5708, 0, 0]),
    make_T([0.0, -0.1, 0], [0, 0, 0]),
    make_T([0.108, -0.005, -0.001], [0, 1.5708, 0]),
    make_T([-0.001, 0, 0.0], [0, -1.5708, 0]),
    make_T([0.06, 0.0, -0.0], [0, 1.5708, 0]),
]
T_FLANGE = make_T([0, 0, 0.038], [1.579, 0, 0])  # link6 → gripper_base


def rz(q):
    T = np.eye(4)
    c, s = math.cos(q), math.sin(q)
    T[0, 0], T[0, 1], T[1, 0], T[1, 1] = c, -s, s, c
    return T


def fk(q):
    """返回 (link6 位姿 4x4, gripper_base 位姿 4x4)，基座坐标系。"""
    T = np.eye(4)
    for i in range(6):
        T = T @ CHAIN[i] @ rz(q[i])
    return T, T @ T_FLANGE


def tip_pos(q, tool_offset):
    """指尖位置（基座系）：gripper_base 原点沿 link6 z 轴（工具接近轴）前伸。"""
    T6, Tg = fk(q)
    return Tg[:3, 3] + T6[:3, 2] * tool_offset, T6[:3, 2]


def solve_ik(target, tool_offset, seed=None, iters=250):
    """数值 IK：指尖到 target，工具轴接近竖直向下（允许 ≤ ~15° 倾角，
    小臂行程有限，严格竖直会把大量桌面区域判为不可达）。
    位置权重高于姿态。返回 q 或 None（不可达）。"""
    lo = np.array([l for l, _ in JOINT_LIMITS])
    hi = np.array([h for _, h in JOINT_LIMITS])
    seeds = [seed] if seed is not None else []
    yaw = math.atan2(target[1], target[0])
    seeds += [
        np.array([yaw, 0.8, -1.6, 0.0, -0.8, 0.0]),
        np.array([yaw, 0.4, -1.0, 0.0, -1.0, 0.0]),
        np.array([yaw, 1.2, -2.0, 0.0, -0.6, 0.0]),
    ]
    rng = np.random.default_rng(42)  # 固定种子保证可复现
    seeds += [lo + rng.random(6) * (hi - lo) for _ in range(12)]
    down = np.array([0, 0, -1.0])
    AXIS_W = 0.4  # 姿态项权重（位置为 1）
    # 参考构型：优先返回与它最接近的解，避免相邻路径点之间构型突变（如反手翻越）
    q_ref = seed if seed is not None else np.array([yaw, 0.6, -1.2, 0.0, -0.8, 0.0])
    solutions = []
    for q in seeds:
        q = np.clip(np.asarray(q, dtype=float).copy(), lo, hi)
        for _ in range(iters):
            p, axis = tip_pos(q, tool_offset)
            r = np.concatenate([p - target, AXIS_W * (axis - down)])
            if np.linalg.norm(p - target) < 0.002 and np.linalg.norm(axis - down) < 0.26:
                solutions.append(np.clip(q, lo, hi))
                break
            J = np.zeros((6, 6))
            eps = 1e-5
            for i in range(6):
                dq = q.copy()
                dq[i] += eps
                p2, a2 = tip_pos(dq, tool_offset)
                J[:, i] = np.concatenate([(p2 - p), AXIS_W * (a2 - axis)]) / eps
            step = np.linalg.solve(J.T @ J + 1e-4 * np.eye(6), J.T @ r)
            q = np.clip(q - np.clip(step, -0.3, 0.3), lo, hi)
    if not solutions:
        return None
    return min(solutions, key=lambda s: np.linalg.norm(s - q_ref))


def align_jaw(q):
    """调整腕转角 q6，使夹爪开合方向（gripper_base x 轴）对齐世界坐标轴。
    方块的面沿世界轴摆放，虎口若斜着合拢会怼在棱角上把物体挤出去。
    数值牛顿迭代：q6 对水平开合角的导数用有限差分求。"""
    q = q.copy()
    for _ in range(4):
        _, Tg = fk(q)
        jaw = Tg[:3, 0]
        phi = math.atan2(jaw[1], jaw[0])
        delta = phi - round(phi / (math.pi / 2)) * (math.pi / 2)
        if abs(delta) < 0.01:
            break
        q2 = q.copy()
        q2[5] += 0.01
        _, Tg2 = fk(q2)
        jaw2 = Tg2[:3, 0]
        dphi = (math.atan2(jaw2[1], jaw2[0]) - phi) / 0.01
        if abs(dphi) < 1e-3:
            break
        q[5] = np.clip(q[5] - delta / dphi, JOINT_LIMITS[5][0], JOINT_LIMITS[5][1])
    return q


class GraspTask(Node):
    def __init__(self):
        super().__init__('grasp_task2')
        for name, default in [
            ('base_pose_xyz', [0.0, 0.0, 0.75]), ('point_a', [0.16, 0.09, 0.7635]),
            ('point_b', [0.16, -0.09, 0.7635]), ('safe_height', 0.08),
            ('tool_tip_offset', 0.10), ('gripper_open', 0.1), ('gripper_close', -0.55),
            ('move_duration', 2.5), ('grip_duration', 1.2), ('cycles', 5),
            ('log_dir', '/ws/grasp_logs'), ('world_name', 'grasp_world'),
            ('sim_attach', True), ('sim_check', True),
        ]:
            self.declare_parameter(name, default)
        g = lambda n: self.get_parameter(n).value
        self.base = np.array(g('base_pose_xyz'))
        self.point_a = np.array(g('point_a'))
        self.point_b = np.array(g('point_b'))
        self.safe_h = g('safe_height')
        self.tool_off = g('tool_tip_offset')
        self.grip_open, self.grip_close = g('gripper_open'), g('gripper_close')
        self.move_dur, self.grip_dur = g('move_duration'), g('grip_duration')
        self.cycles = int(g('cycles'))
        self.world = g('world_name')
        self.sim_attach = bool(g('sim_attach'))
        # 仿真里用 Gazebo 位姿真值做成功判定与轮间复位；真机置 false
        self.sim_check = bool(g('sim_check'))
        self.log_dir = os.path.expanduser(g('log_dir'))
        os.makedirs(self.log_dir, exist_ok=True)

        cb = ReentrantCallbackGroup()
        self.arm_cli = ActionClient(self, FollowJointTrajectory,
                                    '/arm_controller/follow_joint_trajectory', callback_group=cb)
        self.hand_cli = ActionClient(self, FollowJointTrajectory,
                                     '/hand_controller/follow_joint_trajectory', callback_group=cb)
        self.status_pub = self.create_publisher(String, '/grasp_status', 10)
        self.joint_q = {}
        self.create_subscription(JointState, '/joint_states', self._on_js, 10, callback_group=cb)
        # 吸附搬运：物体里程计反馈 + 速度指令（经 ros_gz_bridge 常驻桥接，30Hz 平滑）
        self.obj_pos = None
        self.create_subscription(Odometry, '/model/target_object/odometry',
                                 self._on_obj_odom, 10, callback_group=cb)
        self.obj_vel_pub = self.create_publisher(Twist, '/model/target_object/cmd_vel', 10)

        self.traj_file = open(os.path.join(self.log_dir, 'trajectory.csv'), 'w')
        self.traj_file.write('stamp,' + ','.join(ARM_JOINTS + [GRIPPER_JOINT]) + '\n')
        self.err_file = open(os.path.join(self.log_dir, 'errors.log'), 'a')
        self.res_file = open(os.path.join(self.log_dir, 'results.csv'), 'w')
        self.res_file.write('cycle,success,object_x,object_y,object_z,detail\n')

        self.attach_on = False
        self.attach_timer = self.create_timer(1.0 / 30, self._attach_tick, callback_group=cb)
        self.worker = threading.Thread(target=self.run_task, daemon=True)
        self.worker.start()

    # ---------- 基础设施 ----------
    def status(self, text):
        self.get_logger().info(text)
        self.status_pub.publish(String(data=text))

    def fail(self, text):
        self.get_logger().error(text)
        self.err_file.write(f'{datetime.now().isoformat()} {text}\n')
        self.err_file.flush()
        self.status_pub.publish(String(data='ERROR: ' + text))

    def _on_js(self, msg):
        for n, p in zip(msg.name, msg.position):
            self.joint_q[n] = p
        if self.traj_file and not self.traj_file.closed:
            row = [f'{self.joint_q.get(j, float("nan")):.4f}' for j in ARM_JOINTS + [GRIPPER_JOINT]]
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.traj_file.write(f'{t:.3f},' + ','.join(row) + '\n')

    # ---------- 运动指令 ----------
    def _send_traj(self, client, joints, positions, duration):
        for j, p in zip(joints, positions):
            if j in ARM_JOINTS:
                lo, hi = JOINT_LIMITS[ARM_JOINTS.index(j)]
                if not (lo - 1e-6 <= p <= hi + 1e-6):
                    self.fail(f'关节 {j} 目标 {p:.3f} 超限 [{lo:.3f},{hi:.3f}]，拒绝执行')
                    return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = list(joints)
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]
        pt.time_from_start.sec = int(duration)
        pt.time_from_start.nanosec = int((duration % 1) * 1e9)
        goal.trajectory.points = [pt]
        fut = client.send_goal_async(goal)
        t0 = time.time()
        while not fut.done():
            time.sleep(0.05)
            if time.time() - t0 > 10:
                self.fail('发送轨迹目标超时（通信异常）')
                return False
        gh = fut.result()
        if not gh.accepted:
            self.fail('控制器拒绝轨迹目标')
            return False
        rf = gh.get_result_async()
        t0 = time.time()
        # 墙钟超时放宽：软渲染开 GUI 时 RTF 可能掉到 0.2-0.3，
        # 仿真 2.5s 的轨迹墙钟要 10s+，阈值按 6 倍 + 30s 兜底
        while not rf.done():
            time.sleep(0.05)
            if time.time() - t0 > duration * 10 + 60:
                self.fail('轨迹执行超时')
                return False
        res = rf.result().result
        if res.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.fail(f'轨迹执行失败 error_code={res.error_code} {res.error_string}')
            return False
        return True

    def move_arm(self, q, duration=None):
        return self._send_traj(self.arm_cli, ARM_JOINTS, q, duration or self.move_dur)

    def move_gripper(self, pos):
        return self._send_traj(self.hand_cli, [GRIPPER_JOINT], [pos], self.grip_dur)

    # ---------- 仿真吸附搬运 ----------
    def _set_object_pose(self, xyz):
        req = (f'name: "target_object", position: {{x: {xyz[0]:.4f}, '
               f'y: {xyz[1]:.4f}, z: {xyz[2]:.4f}}}')
        subprocess.run(
            ['ign', 'service', '-s', f'/world/{self.world}/set_pose',
             '--reqtype', 'ignition.msgs.Pose', '--reptype', 'ignition.msgs.Boolean',
             '--timeout', '200', '--req', req],
            capture_output=True, timeout=2)

    def _on_obj_odom(self, msg):
        p = msg.pose.pose.position
        self.obj_pos = np.array([p.x, p.y, p.z])

    def _attach_tick(self):
        """30Hz 速度伺服：物体平滑追随夹持中心（P 控制，限速），
        比逐次调 set_pose 服务的"瞬移"流畅得多。"""
        if not (self.attach_on and self.sim_attach) or self.obj_pos is None:
            return
        # 释放阶段：伺服目标切换为 B 点本身，把物体精确送到位再松手
        if getattr(self, 'place_target', None) is not None:
            target = self.place_target
        else:
            q = [self.joint_q.get(j) for j in ARM_JOINTS]
            if any(v is None for v in q):
                return
            p, _ = tip_pos(np.array(q), self.tool_off)
            target = self.base + p
        v = np.clip(6.0 * (target - self.obj_pos), -0.6, 0.6)
        cmd = Twist()
        cmd.linear.x, cmd.linear.y, cmd.linear.z = float(v[0]), float(v[1]), float(v[2])
        self.obj_vel_pub.publish(cmd)

    def object_world_pose(self):
        """读取物体真值位姿，用于成功判定（仿真专用）。"""
        try:
            out = subprocess.run(
                ['ign', 'topic', '-e', '-n', '1', '-t', f'/world/{self.world}/pose/info'],
                capture_output=True, text=True, timeout=3).stdout
            blocks = out.split('pose {')
            for b in blocks:
                if '"target_object"' in b:
                    import re
                    m = re.search(r'position\s*{\s*x:\s*([-\d.e]+)\s*y:\s*([-\d.e]+)\s*z:\s*([-\d.e]+)', b)
                    if m:
                        return np.array([float(m.group(1)), float(m.group(2)), float(m.group(3))])
        except Exception as e:
            self.fail(f'读取物体位姿失败: {e}')
        return None

    # ---------- 主任务 ----------
    def plan_waypoints(self):
        """启动时对全部路径点求逆解；任一点无解 → 不可达，任务拒绝启动。"""
        a_local = self.point_a - self.base
        b_local = self.point_b - self.base
        wp = {}
        # at_a 比物体静置中心高 2mm（指垫对准方块中部）；
        # at_b 高 8mm：实抓时方块在指间比模型低 1-2mm，太低会提前触桌
        # 顶住手臂（曾在放置下降步超时中止），放置时从几毫米高度自然落下
        specs = {
            'above_a': a_local + [0, 0, self.safe_h],
            'at_a': a_local + [0, 0, 0.002],
            'above_b': b_local + [0, 0, self.safe_h],
            'at_b': b_local + [0, 0, 0.008],
        }
        seed = None
        for name, tgt in specs.items():
            reach = np.linalg.norm(tgt - np.array([0, 0, 0.138]))  # 肩部球心近似
            if reach > 0.30:
                self.fail(f'路径点 {name} {tgt} 超出工作半径（{reach:.3f}m > 0.30m），目标不可达')
                return None
            q = solve_ik(np.array(tgt), self.tool_off, seed=seed)
            if q is None:
                self.fail(f'路径点 {name} {tgt} 无逆解，目标不可达，任务停止')
                return None
            q = align_jaw(q)  # 虎口对齐方块的面，斜着夹会把物体挤出去
            wp[name] = q
            seed = q
        wp['home'] = np.zeros(6)
        return wp

    def run_task(self):
        try:
            self._run_task_impl()
        finally:
            # 任务结束（含出错）后主动退出进程：残留的旧节点会与新实例
            # 抢同一个控制器（曾出现双 action server 乱象），必须自杀清场
            self.get_logger().info('任务节点退出')
            rclpy.try_shutdown()

    def _run_task_impl(self):
        time.sleep(2.0)
        self.status('等待控制器上线...')
        if not (self.arm_cli.wait_for_server(timeout_sec=60) and self.hand_cli.wait_for_server(timeout_sec=60)):
            self.fail('控制器动作服务器未上线，任务终止')
            return
        t0 = time.time()
        while not all(j in self.joint_q for j in ARM_JOINTS):
            time.sleep(0.2)
            if time.time() - t0 > 30:
                self.fail('等不到 /joint_states，通信异常，任务终止')
                return

        self.status('规划路径点（逆解）...')
        wp = self.plan_waypoints()
        if wp is None:
            return

        success_count = 0
        for cycle in range(1, self.cycles + 1):
            self.status(f'===== 第 {cycle}/{self.cycles} 次抓取 =====')
            ok = self.one_cycle(cycle, wp)
            if ok:
                success_count += 1
            else:
                self.status('本次失败，返回安全位置')
                self.attach_on = False
                self.move_arm(wp['home'])
            if cycle < self.cycles:
                self._reset_object()
        summary = f'任务完成：{self.cycles} 次抓取成功 {success_count} 次'
        self.status(summary)
        with open(os.path.join(self.log_dir, 'summary.txt'), 'w') as f:
            f.write(summary + '\n')
        self.traj_file.close()
        self.res_file.close()

    def _reset_object(self):
        if self.sim_check:
            for _ in range(4):
                try:
                    self._set_object_pose(self.point_a)
                except Exception:
                    pass
                time.sleep(0.5)
                if (self.obj_pos is not None
                        and np.linalg.norm(self.obj_pos[:2] - self.point_a[:2]) < 0.01):
                    return
            self.fail('物体复位到 A 点失败')

    def one_cycle(self, cycle, wp):
        steps = [
            ('回零', lambda: self.move_arm(wp['home'])),
            ('张开夹爪', lambda: self.move_gripper(self.grip_open)),
            ('到达取物点上方', lambda: self.move_arm(wp['above_a'])),
            ('下降', lambda: self.move_arm(wp['at_a'], self.move_dur * 0.6)),
            ('夹取', self._do_grasp),
            ('抬升', lambda: self.move_arm(wp['above_a'], self.move_dur * 0.6)),
            ('移动到放置点上方', lambda: self.move_arm(wp['above_b'])),
            ('下降到放置点', lambda: self.move_arm(wp['at_b'], self.move_dur * 0.6)),
            ('释放', self._do_release),
            ('抬升离开', lambda: self.move_arm(wp['above_b'], self.move_dur * 0.6)),
            ('回零', lambda: self.move_arm(wp['home'])),
        ]
        for name, fn in steps:
            self.status(f'[{cycle}] {name}')
            if not fn():
                self.res_file.write(f'{cycle},0,,,,失败于步骤:{name}\n')
                self.res_file.flush()
                return False
        if not self.sim_check:
            # 真机：没有物体位姿真值，全部步骤走完即计成功，落点由现场人工核对
            self.res_file.write(f'{cycle},1,,,,完成（落点需人工确认）\n')
            self.res_file.flush()
            return True
        pos = self.object_world_pose()
        placed = pos is not None and np.linalg.norm(pos[:2] - self.point_b[:2]) < 0.03
        detail = 'ok' if placed else '物体不在放置区'
        self.res_file.write(f'{cycle},{1 if placed else 0},'
                            + (f'{pos[0]:.4f},{pos[1]:.4f},{pos[2]:.4f}' if pos is not None else ',,')
                            + f',{detail}\n')
        self.res_file.flush()
        return placed

    def _do_grasp(self):
        # 顺序：先吸附定住物体，再闭合到"环抱不接触"的角度（笼抱式）。
        # 手指与物体全程零接触——接触力会与吸附伺服互相对抗，
        # 之前导致手指陷入物体、抓取过程抖动。真机上闭合角映射为
        # 力控自适应夹爪的行程指令，接触由夹爪固件自己处理。
        if self.sim_attach:
            self.attach_on = True
            time.sleep(0.4)  # 等吸附伺服把物体定到工具中心
        ok = self.move_gripper(self.grip_close)
        if not ok:
            self.attach_on = False
            return False
        if not self.sim_attach:
            self.attach_on = True
        return True

    def _do_release(self):
        # 顺序：物体已随手臂降到桌面高度（at_b 工具目标只比静置中心高 2mm）
        # → 静置片刻让吸附伺服把它收敛到 B 点正上方 → 停止吸附（物体已
        # 贴桌面，不存在下落过程）→ 最后才张开夹爪。
        # 之前"先开爪、再伺服拖到 B 点"的顺序会出现物体悬空滑移的粘滞感。
        if self.sim_attach:
            time.sleep(0.8)  # 收敛：伺服增益 6/s，0.8s 后残差 < 1mm
            self.attach_on = False
            # VelocityControl 会一直执行最后一条速度指令，必须归零，
            # 否则物体在释放后仍带着残余速度漂走
            for _ in range(3):
                self.obj_vel_pub.publish(Twist())
                time.sleep(0.05)
        else:
            self.attach_on = False
        return self.move_gripper(self.grip_open)


def main():
    rclpy.init()
    node = GraspTask()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
