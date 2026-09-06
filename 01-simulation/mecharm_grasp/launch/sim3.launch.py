"""一键启动抓取流程 3：反向对角搬运。

复用已验证的 grasp_task 控制算法，通过节点重命名加载 grasp_task3 参数，
并使用独立世界和日志目录，不影响流程 1/2。
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('mecharm_grasp')
    world = os.path.join(pkg, 'worlds', 'theWorld3.sdf')
    xacro_file = os.path.join(pkg, 'urdf', 'arm_model.xacro')
    controllers = os.path.join(pkg, 'config', 'controllers.yaml')
    params = os.path.join(pkg, 'config', 'grasp3.yaml')

    gui = LaunchConfiguration('gui')
    desc_pkg = get_package_share_directory('mycobot_description')
    share_roots = [os.path.dirname(pkg), os.path.dirname(desc_pkg)]
    resource_env = SetEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH',
        ':'.join(share_roots + [os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')]))

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' controllers_file:=', controllers]),
        value_type=str)

    gz_gui = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world],
        output='screen', condition=IfCondition(gui))
    gz_headless = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '-s', world],
        output='screen', condition=UnlessCondition(gui))

    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/model/target_object/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/model/target_object/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
        ],
        output='screen')

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen')

    spawn = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-topic', 'robot_description', '-name', 'mecharm',
                   '-x', '0', '-y', '0', '-z', '0.75'],
        output='screen')

    def spawner(name):
        return Node(package='controller_manager', executable='spawner',
                    arguments=[name, '--controller-manager-timeout', '60'],
                    output='screen')

    jsb = spawner('joint_state_broadcaster')
    arm = spawner('arm_controller')
    hand = spawner('hand_controller')

    grasp = Node(
        package='mecharm_grasp', executable='grasp_task', name='grasp_task3',
        parameters=[params], output='screen',
        condition=IfCondition(LaunchConfiguration('task')))

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('task', default_value='true'),
        resource_env,
        gz_gui, gz_headless,
        clock_bridge, rsp, spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[arm, hand])),
        RegisterEventHandler(OnProcessExit(target_action=arm, on_exit=[grasp])),
    ])