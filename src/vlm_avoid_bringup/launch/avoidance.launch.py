"""感知-决策-执行管线（不含仿真器）。

参数优先级（后者覆盖前者）：
    config/avoidance.yaml  <  config/tunable.json  <  launch 参数

用户日常调参只改 config/tunable.json（colcon 是 symlink-install，改完重启 launch 即可）。
"""
import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _load_tunables(path: str) -> dict:
    """读 tunable.json；缺文件就当空配置，格式错直接报出来（别静默忽略）。"""
    if not os.path.isfile(path):
        print(f'[avoidance.launch] 未找到 {path}，使用 avoidance.yaml 的默认值')
        return {}
    with open(path, encoding='utf-8') as f:
        try:
            data = json.load(f)
        except json.JSONDecodeError as exc:
            raise RuntimeError(f'tunable.json 不是合法 JSON：{exc}') from exc
    if not isinstance(data, dict):
        raise RuntimeError('tunable.json 顶层必须是对象')
    return data


def _clean(section: dict) -> dict:
    """去掉 _comment 之类的注释键，避免把未声明的参数塞给节点。"""
    return {k: v for k, v in section.items() if not k.startswith('_')}


def generate_launch_description() -> LaunchDescription:
    bringup = get_package_share_directory('vlm_avoid_bringup')
    config = os.path.join(bringup, 'config', 'avoidance.yaml')
    tunables = _load_tunables(os.path.join(bringup, 'config', 'tunable.json'))

    use_sim_time = LaunchConfiguration('use_sim_time')
    publish_debug_image = ParameterValue(
        LaunchConfiguration('publish_debug_image'), value_type=bool)
    base = [config, {
        'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
    }]

    def node_params(name: str) -> list:
        return base + [_clean(tunables.get(name, {}))]

    vlm_node = Node(
        package='vlm_perception',
        executable='vlm_node',
        name='vlm_obstacle_node',
        output='screen',
        parameters=node_params('vlm_obstacle_node') + [{
            'image_topic': LaunchConfiguration('image_topic'),
            'vlm_url': LaunchConfiguration('vlm_url'),
            'api_style': LaunchConfiguration('api_style'),
            'model': LaunchConfiguration('vlm_model'),
            'publish_debug_image': publish_debug_image,
        }],
    )

    depth_node = Node(
        package='vlm_perception',
        executable='depth_sectors_node',
        name='depth_sectors_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_depth')),
        parameters=node_params('depth_sectors_node') + [{
            'depth_topic': LaunchConfiguration('depth_topic'),
        }],
    )

    scan_node = Node(
        package='vlm_perception',
        executable='scan_sectors_node',
        name='scan_sectors_node',
        output='screen',
        parameters=node_params('scan_sectors_node') + [{
            'scan_topic': LaunchConfiguration('scan_topic'),
        }],
    )

    slam_node = Node(
        package='vlm_control',
        executable='slam_node',
        name='slam_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_slam')),
        parameters=node_params('slam_node'),
    )

    planner_node = Node(
        package='vlm_control',
        executable='path_planner_node',
        name='path_planner_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_planner')),
        parameters=node_params('path_planner_node') + [{
            'map_source': LaunchConfiguration('map_source'),
            'frame_id': LaunchConfiguration('plan_frame'),
            'map_file': LaunchConfiguration('map_file'),
            'map_offset_x': ParameterValue(LaunchConfiguration('spawn_x'),
                                           value_type=float),
            'map_offset_y': ParameterValue(LaunchConfiguration('spawn_y'),
                                           value_type=float),
        }],
    )

    imu_node = Node(
        package='vlm_control',
        executable='imu_state_node',
        name='imu_state_node',
        output='screen',
        parameters=node_params('imu_state_node'),
    )

    controller = Node(
        package='vlm_control',
        executable='avoidance_controller',
        name='avoidance_controller',
        output='screen',
        parameters=node_params('avoidance_controller') + [{
            'require_imu': ParameterValue(LaunchConfiguration('require_imu'),
                                          value_type=bool),
            'enable': ParameterValue(LaunchConfiguration('enable_control'),
                                     value_type=bool),
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', os.path.join(bringup, 'config', 'avoid.rviz')],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('vlm_url', default_value='http://127.0.0.1:11434'),
        DeclareLaunchArgument('api_style', default_value='ollama'),
        DeclareLaunchArgument('vlm_model', default_value='qwen2.5vl:3b'),
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('depth_topic', default_value='/camera/depth/image_raw'),
        DeclareLaunchArgument('enable_slam', default_value='false',
                              description='启动激光+IMU+轮速+RGB-D 的 SLAM，用来修里程计漂移'),
        DeclareLaunchArgument('odom_topic', default_value='/odom',
                              description='位姿来源：SLAM 模式传 /slam/odom'),
        DeclareLaunchArgument('plan_frame', default_value='odom',
                              description='路径/地图的坐标系：RTAB-Map 模式传 map'),
        DeclareLaunchArgument('map_source', default_value='file',
                              description="A* 地图来源：file=离线地图 / topic=SLAM 实时 /map"),
        DeclareLaunchArgument('enable_planner', default_value='true',
                              description='A* 全局规划；关掉则退化为直线奔向目标'),
        DeclareLaunchArgument(
            'map_file',
            default_value=os.path.join(bringup, 'config', 'warehouse_map.json')),
        # 出生点（世界坐标）——odom 从出生点算起，A* 地图是世界坐标，需要这个平移
        DeclareLaunchArgument('spawn_x', default_value='1.5'),
        DeclareLaunchArgument('spawn_y', default_value='1.5'),
        DeclareLaunchArgument('scan_topic', default_value='/scan',
                              description='激光雷达话题；没有雷达时留空即可（节点收不到数据）'),
        DeclareLaunchArgument('enable_depth', default_value='true',
                              description='没有深度相机就设 false（退化为纯 VLM+IMU）'),
        DeclareLaunchArgument('publish_debug_image', default_value='true'),
        DeclareLaunchArgument('enable_control', default_value='true'),
        DeclareLaunchArgument('require_imu', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        # rviz 放最前：Gazebo 加载世界/模型时会吃满 CPU，rviz 的 Qt+GL 初始化
        # 如果排在后面，窗口要等很久才出来
        rviz, vlm_node, depth_node, scan_node, imu_node, slam_node,
        planner_node, controller,
    ])
