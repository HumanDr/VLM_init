"""Gazebo 资产静态校验：不需要显卡、不需要真的跑仿真。

`gz sdf -p robot.urdf` 会把 URDF 转成 SDF，顺带验证所有 <gazebo> 扩展、
传感器和插件标签是否被 Gazebo 认出来——这是"资产写错了"最快的一类检查。
`gz sdf -k world` 校验 world 文件语法。

没装 Gazebo 时自动跳过。
"""
import os
import shutil
import subprocess

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
URDF = os.path.join(ROOT, 'src/vlbot_description/urdf/vlbot.urdf')
WORLD = os.path.join(ROOT, 'src/vlbot_gazebo/worlds/corridor.world')
WAREHOUSE = os.path.join(ROOT, 'src/vlbot_gazebo/worlds/warehouse.world')
GZ = shutil.which('gz')

pytestmark = pytest.mark.skipif(GZ is None, reason='gazebo 未安装（scripts/install_gazebo.sh）')


def _env(tmp_path):
    """Gazebo 会往 $HOME/.gazebo 写缓存；HOME 只读时会直接 abort。"""
    env = dict(os.environ)
    env['HOME'] = str(tmp_path)
    return env


def test_urdf_converts_to_sdf(tmp_path):
    r = subprocess.run([GZ, 'sdf', '-p', URDF], capture_output=True, text=True,
                       env=_env(tmp_path), timeout=120)
    assert r.returncode == 0, r.stderr[-2000:]
    sdf = r.stdout
    assert sdf.strip(), '转换结果为空'

    # ROS2 插件必须全部被识别
    for plugin in ('libgazebo_ros_diff_drive.so', 'libgazebo_ros_camera.so',
                   'libgazebo_ros_imu_sensor.so',
                   'libgazebo_ros_joint_state_publisher.so'):
        assert plugin in sdf, f'URDF 里缺少 {plugin}'

    # 相机 / 深度 / IMU 三个传感器
    for kind in ("type='camera'", "type='depth'", "type='imu'", "type='ray'"):
        assert kind in sdf, f'URDF 里缺少传感器 {kind}'

    # 激光雷达
    assert 'libgazebo_ros_ray_sensor.so' in sdf
    assert '<frame_name>laser_link</frame_name>' in sdf

    # 差速驱动的关键参数
    assert 'wheel_separation' in sdf and 'wheel_diameter' in sdf
    assert 'robot_base_frame' in sdf

    # 相机插件必须是 ROS2 的 snake_case 标签（camelCase 会被静默忽略，
    # 导致话题变成 /camera/<传感器名>/image_raw —— 这是踩过的坑）
    assert '<camera_name>camera</camera_name>' in sdf
    assert '<camera_name>camera_depth</camera_name>' in sdf
    assert '<frame_name>camera_optical_frame</frame_name>' in sdf
    assert 'imageTopicName' not in sdf and 'frameName' not in sdf


def test_world_is_valid(tmp_path):
    r = subprocess.run([GZ, 'sdf', '-k', WORLD], capture_output=True, text=True,
                       env=_env(tmp_path), timeout=120)
    assert r.returncode == 0, (r.stdout + r.stderr)[-2000:]


def test_world_contains_expected_obstacles():
    with open(WORLD, encoding='utf-8') as f:
        text = f.read()
    for name in ('red_box', 'blue_bin', 'yellow_box', 'person',
                 'wall_left', 'wall_right', 'ground_plane'):
        assert f'name="{name}"' in text, f'world 缺少 {name}'


def test_warehouse_world_is_valid(tmp_path):
    r = subprocess.run([GZ, 'sdf', '-k', WAREHOUSE], capture_output=True, text=True,
                       env=_env(tmp_path), timeout=120)
    assert r.returncode == 0, (r.stdout + r.stderr)[-2000:]


def test_warehouse_has_rooms_and_goal_marker():
    with open(WAREHOUSE, encoding='utf-8') as f:
        text = f.read()
    for name in ('wall_south', 'wall_east', 'partition_a', 'partition_b',
                 'partition_c', 'red_box', 'blue_bin', 'person', 'goal_marker'):
        assert f'name="{name}"' in text, f'warehouse 缺少 {name}'
    assert 'goal_marker' in text
