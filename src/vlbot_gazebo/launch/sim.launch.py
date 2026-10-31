"""启动 Gazebo Classic + vlbot 机器人。

用法：
    ros2 launch vlbot_gazebo sim.launch.py                  # 带 GUI
    ros2 launch vlbot_gazebo sim.launch.py gui:=false       # 无 GUI（服务器/纯算法调试）
    ros2 launch vlbot_gazebo sim.launch.py x:=0.6 y:=0.0    # 指定初始位姿
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    vlbot_gazebo_share = get_package_share_directory('vlbot_gazebo')
    vlbot_desc_share = get_package_share_directory('vlbot_description')

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    pause = LaunchConfiguration('pause')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(vlbot_gazebo_share, 'worlds', 'warehouse.world'),
            description='SDF world 文件'),
        DeclareLaunchArgument('gui', default_value='true', description='是否启动 Gazebo 客户端'),
        DeclareLaunchArgument('pause', default_value='false'),
        DeclareLaunchArgument('x', default_value='1.5'),
        DeclareLaunchArgument('y', default_value='1.5'),
        DeclareLaunchArgument('yaw', default_value='0.0'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')),
            launch_arguments={
                'world': world,
                'gui': gui,
                'pause': pause,
                'verbose': 'true',
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(vlbot_desc_share, 'launch', 'description.launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items(),
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_vlbot',
            output='screen',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'vlbot',
                '-x', x, '-y', y, '-z', '0.06', '-Y', yaw,
            ],
        ),
    ])
