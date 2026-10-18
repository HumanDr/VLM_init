"""启动文件冒烟测试：确保每个 launch 文件都能被 launch 框架加载并生成。

sim.launch.py 依赖 gazebo_ros；未安装时跳过（其余必须通过）。
"""
import importlib.util
import os

import pytest
from ament_index_python.packages import get_package_share_directory

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

LAUNCH_FILES = [
    'src/vlm_avoid_bringup/launch/avoidance.launch.py',
    'src/vlm_avoid_bringup/launch/mock_demo.launch.py',
    'src/vlm_avoid_bringup/launch/sim_avoid.launch.py',
    'src/vlbot_description/launch/description.launch.py',
    'src/vlbot_gazebo/launch/sim.launch.py',
]


def _has_package(name: str) -> bool:
    try:
        get_package_share_directory(name)
        return True
    except Exception:  # noqa: BLE001
        return False


@pytest.mark.parametrize('rel_path', LAUNCH_FILES)
def test_launch_file_loads(rel_path: str) -> None:
    needs = 'gazebo_ros' if 'vlbot_gazebo' in rel_path else None
    if needs and not _has_package(needs):
        pytest.skip(f'{needs} 未安装（先跑 scripts/install_deps.sh）')

    path = os.path.join(ROOT, rel_path)
    assert os.path.isfile(path), path
    spec = importlib.util.spec_from_file_location(
        'launch_under_test_' + os.path.basename(rel_path).replace('.', '_'), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    assert description is not None
    assert len(description.entities) > 0


def test_avoidance_yaml_covers_nodes() -> None:
    """参数文件里的节点名必须与控制器实际节点名一致，否则参数静默失效。"""
    import yaml
    cfg_path = os.path.join(ROOT, 'src/vlm_avoid_bringup/config/avoidance.yaml')
    with open(cfg_path, encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    for node in ('vlm_obstacle_node', 'depth_sectors_node',
                 'imu_state_node', 'avoidance_controller'):
        assert node in cfg, f'配置缺少 {node}'
    assert 'ros__parameters' in cfg['avoidance_controller']
