"""无 Gazebo 联调：假相机 + Mock VLM + 完整管线。

用途：还没装 Gazebo、GPU 驱动没修好、或只想快速验证控制策略时，
一条命令就能看到 /cmd_vel 随"画面变化"而变。

    ros2 launch vlm_avoid_bringup mock_demo.launch.py
    # 另开终端切场景：
    ros2 topic pub -1 /test/scene std_msgs/String "{data: center}"
"""
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    bringup = get_package_share_directory('vlm_avoid_bringup')
    port = LaunchConfiguration('mock_port')

    mock_vlm = ExecuteProcess(
        cmd=[sys.executable, '-m', 'vlm_perception.mock_vlm_server',
             '--port', port],
        output='screen',
        name='mock_vlm_server',
    )

    fake_camera = Node(
        package='vlm_perception',
        executable='fake_camera',
        name='fake_camera',
        output='screen',
        parameters=[{
            'scene': LaunchConfiguration('scene'),
            'rate_hz': 10.0,
            'publish_depth': True,
            'publish_imu': True,
        }],
    )

    avoidance = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup, 'launch', 'avoidance.launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'vlm_url': LaunchConfiguration('vlm_url'),
            'api_style': 'ollama',
            'vlm_model': 'mock-vlm',
            'enable_depth': 'true',
            'publish_debug_image': 'true',
            'rviz': LaunchConfiguration('rviz'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('mock_port', default_value='11435'),
        DeclareLaunchArgument('vlm_url', default_value='http://127.0.0.1:11435'),
        DeclareLaunchArgument('scene', default_value='free'),
        DeclareLaunchArgument('rviz', default_value='false'),
        mock_vlm, fake_camera, avoidance,
    ])
