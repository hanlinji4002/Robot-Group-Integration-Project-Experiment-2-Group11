# 【讲解】安装规则，colcon build 时执行。三件事：
# 1) packages=['mecharm_grasp']：把 mecharm_grasp/ 目录当 Python 模块装进去（里面是任务节点）。
# 2) data_files：把 launch/、model/、config/ 里的文件复制到 install/share/mecharm_grasp/ 下，
#    目标子目录固定叫 launch、urdf、worlds、config —— launch 文件运行时就从这几个子目录找文件。
# 3) entry_points：把命令名映射到函数。grasp_task -> ros_node.py 的 main()，
#    所以 launch 里 executable='grasp_task' 启动的就是任务节点。
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
            'grasp_task2 = mecharm_grasp.ros_node2:main',
            'grasp_task3 = mecharm_grasp.ros_node3:main',
            'grasp_task_Qi_Chu = mecharm_grasp.ros_node_Qi_Chu:main',
            # 标定脚本在 config/ 下，source 工作区后 python3 config/calib_*.py 直接运行
        ],
    },
)
