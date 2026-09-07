"""真机（Jetson + mechArm 270）定点抓取一键启动。

启动：真机驱动（mecharm_real/real_driver，经 TCP 调臂内 arm_server.py）
    + 任务节点（mecharm_grasp/grasp_task，与仿真同一份代码）。
用法：
  ros2 launch mecharm_real real.launch.py                   # 5 次抓取
  ros2 launch mecharm_real real.launch.py cycles:=1         # 先单次低速验证
  ros2 launch mecharm_real real.launch.py host:=10.42.0.89  # 指定臂内服务地址
  软件急停（另开终端）：ros2 topic pub --once /soft_stop std_msgs/msg/Bool "data: true"
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    params = os.path.join(get_package_share_directory('mecharm_real'), 'config', 'real.yaml')
    cycles = LaunchConfiguration('cycles')
    host = LaunchConfiguration('host')

    driver = Node(package='mecharm_real', executable='real_driver', output='screen',
                  parameters=[params, {'arm_host': ParameterValue(host, value_type=str)}])
    task = Node(package='mecharm_grasp', executable='grasp_task', output='screen',
                parameters=[params, {'cycles': ParameterValue(cycles, value_type=int)}])

    return LaunchDescription([
        DeclareLaunchArgument('cycles', default_value='5'),
        DeclareLaunchArgument('host', default_value='10.42.0.89'),
        driver,
        # 驱动先连上臂内服务，任务节点延后启动
        TimerAction(period=4.0, actions=[task]),
    ])
