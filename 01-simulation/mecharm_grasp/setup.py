import os
from glob import glob
from setuptools import setup

# robotic experiment2/01-simulation/mecharm_grasp：独立可编译的仿真包。
# 标准 ament_python 布局，不做目录映射。数据文件按用途放在 model/ config/ launch/，
# 安装到 share/ 时用 urdf / worlds / config / launch 子目录，launch 文件按这些子目录找文件。
package_name = 'mecharm_grasp'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'),   glob('model/*.xacro')),
        (os.path.join('share', package_name, 'worlds'), glob('model/*.sdf')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotic-group',
    maintainer_email='1990292743abc@gmail.com',
    description='mechArm 270 定点抓取实验（仿真阶段）：Gazebo Fortress 仿真的任务控制包',
    license='MIT',
    entry_points={
        'console_scripts': [
            'grasp_task = mecharm_grasp.ros_node:main',
            # 标定脚本在 config/ 下，source 工作区后 python3 config/calib_*.py 直接运行
        ],
    },
)
