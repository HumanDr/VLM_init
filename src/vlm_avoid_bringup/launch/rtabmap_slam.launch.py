"""RTAB-Map SLAM：RGB-D + 激光雷达 + IMU 的图优化 SLAM（主力方案）。

选它的理由见 README 的"定位与 SLAM 实验记录"：
  * ROS2 Humble 上维护最活跃、开箱即用的多传感器 SLAM；
  * 支持 RGB-D 视觉 + 激光 + IMU 融合，自带回环检测与位姿图优化；
  * 输出标准的 `/map`（OccupancyGrid）和 `map->odom` TF，下游零改动。

它需要 apt 安装（`bash scripts/install_slam.sh`）。没装时用 `localization:=slam`
跑仓库自带的内置 SLAM（零依赖，精度略低）。

发布（都很标准）：
    /map                   OccupancyGrid（map 系）
    /odom                  RTAB-Map 视觉里程计（未修正）
    TF map -> odom -> base_footprint
另外启动 tf_pose_bridge，把 `map -> base_footprint` 转成 `/slam/odom` 给控制器用。
"""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    try:
        rtabmap_launch = os.path.join(get_package_share_directory('rtabmap_launch'),
                                      'launch', 'rtabmap.launch.py')
    except Exception:  # noqa: BLE001 - 没装 rtabmap 时给出清晰报错
        rtabmap_launch = None

    # 轮式里程计 -> /odom + TF odom->base（RTAB-Map 的里程计输入）
    odom_bridge = Node(
        package='vlm_control',
        executable='wheel_odom_bridge',
        name='wheel_odom_bridge',
        output='screen',
        parameters=[{'wheel_odom_topic': LaunchConfiguration('wheel_odom_topic'),
                     'odom_topic': '/odom',
                     'frame_id': LaunchConfiguration('odom_frame'),
                     'base_frame': LaunchConfiguration('base_frame')}],
    )

    bridge = Node(
        package='vlm_control',
        executable='tf_pose_bridge',
        name='tf_pose_bridge',
        output='screen',
        parameters=[{
            'target_frame': LaunchConfiguration('map_frame'),
            'source_frame': LaunchConfiguration('base_frame'),
            'odom_topic': '/slam/odom',
        }],
    )

    args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rgb_topic', default_value='/camera_depth/image_raw'),
        DeclareLaunchArgument('depth_topic',
                              default_value='/camera_depth/depth/image_raw'),
        DeclareLaunchArgument('camera_info_topic',
                              default_value='/camera_depth/camera_info'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('imu_topic', default_value='/imu/data'),
        DeclareLaunchArgument('wheel_odom_topic', default_value='/wheel_odom'),
        DeclareLaunchArgument('base_frame', default_value='base_footprint'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument(
            'odometry', default_value='wheel',
            description='wheel=轮式里程计（推荐，仿真里不会失锁）; '
                        'icp=激光 ICP; visual=RGB-D 视觉'),
    ]

    if rtabmap_launch is None:
        from launch.actions import LogInfo
        return LaunchDescription(args + [bridge, LogInfo(msg=(
            '[警告] 没有找到 rtabmap_launch：请先 bash scripts/install_slam.sh，'
            '或用 localization:=slam 走内置 SLAM'))])

    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rtabmap_launch),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'rgb_topic': LaunchConfiguration('rgb_topic'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'scan_topic': LaunchConfiguration('scan_topic'),
            'imu_topic': LaunchConfiguration('imu_topic'),
            'frame_id': LaunchConfiguration('base_frame'),
            'odom_frame_id': LaunchConfiguration('odom_frame'),
            'map_frame_id': LaunchConfiguration('map_frame'),
            'subscribe_scan': 'true',
            'visual_odometry': PythonExpression(
                ["'", LaunchConfiguration('odometry'), "' == 'visual'"]),
            'icp_odometry': PythonExpression(
                ["'", LaunchConfiguration('odometry'), "' == 'icp'"]),
            # wheel 模式：不跑 odom 节点，直接用 /odom（由 wheel_odom_bridge 提供）
            'odom_topic': '/odom',
            'approx_sync': 'true',          # 图像/深度/雷达时间戳不完全对齐
            'rtabmap_viz': 'false',         # 无 GUI（服务器/无显示器也能跑）
            'rviz': 'false',
            'gui': 'false',
            'publish_tf_map': 'true',
            # 关键：rtabmap 节点在 /rtabmap 命名空间下，map_topic 默认 = /rtabmap/map。
            # 用绝对名把它钉到 /map，下游（A* / rviz）才能拿到。
            'map_topic': '/map',
            'wait_for_transform': '0.5',
        }.items(),
    )

    return LaunchDescription(args + [odom_bridge, rtabmap, bridge])
