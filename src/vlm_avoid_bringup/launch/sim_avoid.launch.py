"""仿真一键启动：Gazebo 世界 + vlbot + 视觉避障管线 + rviz2。

    ros2 launch vlm_avoid_bringup sim_avoid.launch.py
    ros2 launch vlm_avoid_bringup sim_avoid.launch.py gui:=false rviz:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    loc = LaunchConfiguration('localization')
    is_slam = PythonExpression(["'", loc, "' == 'slam'"])
    is_rtabmap = PythonExpression(["'", loc, "' == 'rtabmap'"])
    is_gt = PythonExpression(["'", loc, "' == 'ground_truth'"])
    odom_topic = PythonExpression(
        ["'/slam/odom' if '", loc, "' in ('slam', 'rtabmap') else '/odom'"])
    map_source = PythonExpression(
        ["'topic' if '", loc, "' in ('slam', 'rtabmap') else 'file'"])
    plan_frame = PythonExpression(
        ["'map' if '", loc, "' == 'rtabmap' else 'odom'"])

    bringup = get_package_share_directory('vlm_avoid_bringup')
    vlbot_gazebo = get_package_share_directory('vlbot_gazebo')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vlbot_gazebo, 'launch', 'sim.launch.py')),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'pause': LaunchConfiguration('pause'),
            'world': LaunchConfiguration('world'),
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
            'scan_topic': LaunchConfiguration('scan_topic'),
            'spawn_x': LaunchConfiguration('x'),
            'spawn_y': LaunchConfiguration('y'),
            'enable_slam': is_slam,
            'odom_topic': odom_topic,
            'map_source': map_source,
            'plan_frame': plan_frame,
            'depth_topic': LaunchConfiguration('depth_topic'),
            'publish_debug_image': LaunchConfiguration('publish_debug_image'),
            'require_imu': LaunchConfiguration('require_imu'),
            'rviz': 'false',
        }.items(),
    )

    # RTAB-Map：RGB-D + 雷达 + IMU 的图优化 SLAM
    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup, 'launch', 'rtabmap_slam.launch.py')),
        condition=IfCondition(is_rtabmap),
        launch_arguments={
            'use_sim_time': 'true',
            'scan_topic': LaunchConfiguration('scan_topic'),
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
        }.items(),
    )

    # 非 RTAB-Map 模式：补一个 map->odom 恒等变换，让 rviz 统一用 map 作固定坐标系
    static_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_static',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        condition=UnlessCondition(is_rtabmap),
    )

    # 真值里程计 -> /odom + TF（只有 ground_truth 模式才启动）
    odom_shim = Node(
        package='vlm_control',
        executable='ground_truth_odom',
        name='ground_truth_odom',
        output='screen',
        condition=IfCondition(is_gt),
        parameters=[{
            'origin_x': ParameterValue(LaunchConfiguration('x'), value_type=float),
            'origin_y': ParameterValue(LaunchConfiguration('y'), value_type=float),
            'origin_yaw': ParameterValue(LaunchConfiguration('yaw'), value_type=float),
        }],
    )

    # Gazebo 进程一 spawn 就立刻起 rviz：它的 Qt/GL 初始化和世界加载并行，
    # 而重的感知管线（VLM→Ollama）排在后面，不会再把 rviz 挤到队尾
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', os.path.join(bringup, 'config', 'avoid.rviz')],
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('pause', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'localization', default_value='rtabmap',
            description='slam=雷达/IMU/轮速/RGB-D 在线建图（默认，无真值）; '
                        'ground_truth=用 Gazebo 真值位姿 + 离线地图（仅用于对比实验）'),
        # 换回旧走廊：world:=<share>/vlbot_gazebo/worlds/corridor.world
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(get_package_share_directory('vlbot_gazebo'),
                                       'worlds', 'warehouse.world')),
        DeclareLaunchArgument('x', default_value='1.5'),
        DeclareLaunchArgument('y', default_value='1.5'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('vlm_url', default_value='http://127.0.0.1:11434'),
        DeclareLaunchArgument('api_style', default_value='ollama'),
        DeclareLaunchArgument('vlm_model', default_value='qwen2.5vl:3b'),
        DeclareLaunchArgument('enable_depth', default_value='true'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        # gazebo_ros_camera 把深度图发到 camera_name + "/depth/image_raw"，
        # 这里 camera_name=camera_depth，所以是 /camera_depth/depth/image_raw
        DeclareLaunchArgument('depth_topic',
                              default_value='/camera_depth/depth/image_raw'),
        DeclareLaunchArgument('publish_debug_image', default_value='true'),
        DeclareLaunchArgument('require_imu', default_value='false'),
        sim, rviz_node, static_map, odom_shim, rtabmap, avoidance,
    ])
