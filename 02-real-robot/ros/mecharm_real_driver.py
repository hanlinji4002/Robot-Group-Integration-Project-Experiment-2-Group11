#!/usr/bin/env python3
"""mechArm 270 真机驱动节点（pymycobot 后端）。

对外提供与仿真阶段 ros2_control 完全相同的 ROS 2 接口，任务节点
grasp_task_node 无需改动即可复用（验收要求：切真机只换设备与参数）：
  - /arm_controller/follow_joint_trajectory   FollowJointTrajectory 动作
  - /hand_controller/follow_joint_trajectory  FollowJointTrajectory 动作
  - /joint_states                             关节状态（默认 10Hz）
  - /soft_stop (std_msgs/Bool, data=true)     软件急停：立即停止并拒绝后续目标

安全设计（对应实验要求）：
  - max_speed 参数限制 send_angles 速度百分比，初次运行用低速模式
  - 每个目标先经关节限位校验（任务节点已查一遍，这里再兜底）
  - 通信异常/超时 → 动作返回失败，任务节点会走"返回安全位置/停止"分支
  - 软件急停触发后 mc.stop()，不释放舵机（防止手臂坠落）

标定项（首次联机按实际情况改 real_grasp_params.yaml）：
  - joint_signs / joint_offsets_deg：URDF 关节方向与真机不一致时逐轴修正
  - gripper_open_value / gripper_close_value：夹爪行程两点标定
"""
import glob
import math
import threading
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

ARM_JOINTS = [
    'joint1_to_base', 'joint2_to_joint1', 'joint3_to_joint2',
    'joint4_to_joint3', 'joint5_to_joint4', 'joint6_to_joint5',
]
GRIPPER_JOINT = 'gripper_controller'

# 关节限位（rad），与 URDF/任务节点一致
JOINT_LIMITS = [
    (-2.792527, 2.792527), (-1.3089, 2.0943), (-3.0543, 1.1344),
    (-2.7052, 2.7052), (-2.0071, 2.0071), (-3.14, 3.14),
]
MAX_JOINT_DEG_S = 120.0  # 官方规格书最大关节速度


def make_robot(port, baud, logger):
    """按 pymycobot 版本尝试可用的 mechArm 驱动类。"""
    import pymycobot
    for cls_name in ('MechArm270', 'MechArm', 'MyCobot280', 'MyCobot'):
        cls = getattr(pymycobot, cls_name, None)
        if cls is None:
            continue
        try:
            mc = cls(port, baud)
            logger.info(f'使用 pymycobot.{cls_name}，端口 {port}@{baud}')
            return mc
        except Exception as e:
            logger.warn(f'{cls_name} 初始化失败: {e}')
    raise RuntimeError('pymycobot 无可用的 mechArm 驱动类')


class MechArmRealDriver(Node):
    def __init__(self):
        super().__init__('mecharm_real_driver')
        for name, default in [
            ('port', ''), ('baud', 115200),
            ('max_speed', 30),            # 速度百分比上限（1-100），低速模式
            ('state_rate', 10.0),
            ('joint_signs', [1.0] * 6),
            ('joint_offsets_deg', [0.0] * 6),
            # 夹爪两点标定：URDF 角度 <-> pymycobot 开度值(0-100)
            ('gripper_open_rad', 0.1), ('gripper_open_value', 95),
            ('gripper_close_rad', -0.25), ('gripper_close_value', 25),
        ]:
            self.declare_parameter(name, default)
        g = lambda n: self.get_parameter(n).value
        self.max_speed = int(g('max_speed'))
        self.signs = list(g('joint_signs'))
        self.offsets = list(g('joint_offsets_deg'))
        self.g_open_rad, self.g_open_val = g('gripper_open_rad'), int(g('gripper_open_value'))
        self.g_close_rad, self.g_close_val = g('gripper_close_rad'), int(g('gripper_close_value'))

        port = g('port') or self._auto_port()
        self.serial_lock = threading.Lock()   # pymycobot 串口不可并发
        self.mc = make_robot(port, int(g('baud')), self.get_logger())
        self.stopped = False
        self.last_angles = None
        self.last_gripper_rad = 0.0

        cb = ReentrantCallbackGroup()
        self.js_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_subscription(Bool, '/soft_stop', self._on_soft_stop, 10, callback_group=cb)
        self.arm_srv = ActionServer(self, FollowJointTrajectory,
                                    '/arm_controller/follow_joint_trajectory',
                                    self._exec_arm, callback_group=cb)
        self.hand_srv = ActionServer(self, FollowJointTrajectory,
                                     '/hand_controller/follow_joint_trajectory',
                                     self._exec_hand, callback_group=cb)
        self.create_timer(1.0 / float(g('state_rate')), self._publish_states, callback_group=cb)
        self.get_logger().info('真机驱动就绪（低速模式 max_speed='
                               f'{self.max_speed}），等待任务节点指令')

    @staticmethod
    def _auto_port():
        cands = sorted(glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*'))
        if not cands:
            raise RuntimeError('找不到机械臂串口（/dev/ttyACM* 或 /dev/ttyUSB*）。'
                               '请检查 USB 连接与供电，或用 port 参数指定')
        return cands[0]

    # ---------- 角度换算 ----------
    def _rad_to_deg(self, q):
        return [math.degrees(v) * s + o for v, s, o in zip(q, self.signs, self.offsets)]

    def _deg_to_rad(self, degs):
        return [math.radians((d - o) / s) for d, s, o in zip(degs, self.signs, self.offsets)]

    def _gripper_rad_to_value(self, rad):
        u = (rad - self.g_close_rad) / (self.g_open_rad - self.g_close_rad)
        v = self.g_close_val + u * (self.g_open_val - self.g_close_val)
        return int(max(0, min(100, round(v))))

    # ---------- 状态发布 ----------
    def _publish_states(self):
        with self.serial_lock:
            try:
                degs = self.mc.get_angles()
            except Exception:
                degs = None
        if not degs or len(degs) != 6:
            return
        self.last_angles = degs
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ARM_JOINTS + [GRIPPER_JOINT]
        js.position = self._deg_to_rad(degs) + [self.last_gripper_rad]
        self.js_pub.publish(js)

    # ---------- 急停 ----------
    def _on_soft_stop(self, msg):
        if msg.data:
            self.stopped = True
            with self.serial_lock:
                try:
                    self.mc.stop()
                except Exception:
                    pass
            self.get_logger().error('软件急停触发：已停止运动，拒绝后续目标'
                                    '（恢复需发布 /soft_stop data:false）')
        else:
            self.stopped = False
            self.get_logger().info('软件急停解除')

    # ---------- 动作执行 ----------
    def _result(self, gh, ok, err=''):
        res = FollowJointTrajectory.Result()
        if ok:
            res.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            gh.succeed()
        else:
            res.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
            res.error_string = err
            self.get_logger().error(f'轨迹执行失败: {err}')
            gh.abort()
        return res

    def _exec_arm(self, gh):
        traj = gh.request.trajectory
        if self.stopped:
            return self._result(gh, False, '处于急停状态')
        if list(traj.joint_names) != ARM_JOINTS or not traj.points:
            return self._result(gh, False, f'非法目标: {traj.joint_names}')
        pt = traj.points[-1]
        q = list(pt.positions)
        for (lo, hi), v, name in zip(JOINT_LIMITS, q, ARM_JOINTS):
            if not (lo - 1e-6 <= v <= hi + 1e-6):
                return self._result(gh, False, f'关节 {name} 目标 {v:.3f} 超限')
        duration = max(0.5, pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9)
        target_deg = self._rad_to_deg(q)
        cur = self.last_angles or target_deg
        delta = max(abs(t - c) for t, c in zip(target_deg, cur))
        # 速度百分比 = 期望角速度 / 最大角速度，夹在 [5, max_speed]
        speed = int(max(5, min(self.max_speed, delta / duration / MAX_JOINT_DEG_S * 100)))
        with self.serial_lock:
            try:
                self.mc.send_angles(target_deg, speed)
            except Exception as e:
                return self._result(gh, False, f'串口通信异常: {e}')
        # 轮询到位（3° 容差），超时按比例放宽
        deadline = time.time() + duration * 3 + 5
        while time.time() < deadline:
            if self.stopped:
                return self._result(gh, False, '执行中被急停')
            if self.last_angles and max(
                    abs(t - c) for t, c in zip(target_deg, self.last_angles)) < 3.0:
                return self._result(gh, True)
            time.sleep(0.1)
        return self._result(gh, False, '到位超时（通信或负载异常）')

    def _exec_hand(self, gh):
        traj = gh.request.trajectory
        if self.stopped:
            return self._result(gh, False, '处于急停状态')
        if list(traj.joint_names) != [GRIPPER_JOINT] or not traj.points:
            return self._result(gh, False, f'非法目标: {traj.joint_names}')
        pt = traj.points[-1]
        rad = float(pt.positions[0])
        value = self._gripper_rad_to_value(rad)
        duration = max(0.5, pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9)
        with self.serial_lock:
            try:
                self.mc.set_gripper_value(value, 50)
            except Exception as e:
                return self._result(gh, False, f'夹爪通信异常: {e}')
        time.sleep(duration)
        self.last_gripper_rad = rad
        return self._result(gh, True)


def main():
    rclpy.init()
    node = MechArmRealDriver()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        with node.serial_lock:
            try:
                node.mc.stop()
            except Exception:
                pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
