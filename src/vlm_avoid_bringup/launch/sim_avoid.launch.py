"""仿真一键启动：Gazebo 世界 + vlbot + 视觉避障管线 + rviz2。

    ros2 launch vlm_avoid_bringup sim_avoid.launch.py
    ros2 launch vlm_avoid_bringup sim_avoid.launch.py gui:=false rviz:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    bringup = get_package_share_directory('vlm_avoid_bringup')
    vlbot_gazebo = get_package_share_directory('vlbot_gazebo')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vlbot_gazebo, 'launch', 'sim.launch.py')),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'pause': LaunchConfiguration('pause'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'yaw': LaunchConfiguration('yaw'),
        }.items(),
    )

    avoidance = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup, 'launch', 'avoidance.launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'vlm_url': LaunchConfiguration('vlm_url'),
            'api_style': LaunchConfiguration('api_style'),
            'vlm_model': LaunchConfiguration('vlm_model'),
            'enable_depth': LaunchConfiguration('enable_depth'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'publish_debug_image': LaunchConfiguration('publish_debug_image'),
            'require_imu': LaunchConfiguration('require_imu'),
            'rviz': LaunchConfiguration('rviz'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('pause', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('x', default_value='0.6'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('vlm_url', default_value='http://127.0.0.1:11434'),
        DeclareLaunchArgument('api_style', default_value='ollama'),
        DeclareLaunchArgument('vlm_model', default_value='qwen2.5vl:3b'),
        DeclareLaunchArgument('enable_depth', default_value='true'),
        # gazebo_ros_camera 把深度图发到 camera_name + "/depth/image_raw"，
        # 这里 camera_name=camera_depth，所以是 /camera_depth/depth/image_raw
        DeclareLaunchArgument('depth_topic',
                              default_value='/camera_depth/depth/image_raw'),
        DeclareLaunchArgument('publish_debug_image', default_value='true'),
        DeclareLaunchArgument('require_imu', default_value='false'),
        sim, avoidance,
    ])
