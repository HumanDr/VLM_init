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
        DeclareLaunchArgument('enable_depth', default_value='true',
                              description='没有深度相机就设 false（退化为纯 VLM+IMU）'),
        DeclareLaunchArgument('publish_debug_image', default_value='true'),
        DeclareLaunchArgument('enable_control', default_value='true'),
        DeclareLaunchArgument('require_imu', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        vlm_node, depth_node, imu_node, controller, rviz,
    ])
