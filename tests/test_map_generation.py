"""从 Gazebo 世界生成栅格地图，并验证 A* 能绕开墙。"""
import importlib.util
import os

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPT = os.path.join(ROOT, 'scripts/make_map_from_world.py')
WORLD = os.path.join(ROOT, 'src/vlbot_gazebo/worlds/warehouse.world')

pytestmark = pytest.mark.skipif(not os.path.isfile(WORLD), reason='world 不存在')


def _load_builder():
    spec = importlib.util.spec_from_file_location('mkm', SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope='module')
def world_map():
    mod = _load_builder()
    data = mod.build(WORLD, resolution=0.1)
    from vlm_control.grid_map import GridMap
    return data, GridMap(data['rows'], data['resolution'], data['origin'])


def test_meta(world_map):
    data, m = world_map
    assert data['resolution'] == pytest.approx(0.1)
    assert m.width > 200 and m.height > 150
    # 26x18 的仓库，占用率应该是个位数百分比
    ratio = data['occupied_cells'] / (m.width * m.height)
    assert 0.01 < ratio < 0.15, ratio


def test_walls_are_occupied(world_map):
    _data, m = world_map
    # 墙 A 在 x=9，y 从 -9 到 2
    for x, y in ((9.0, 0.0), (9.0, -5.0), (9.0, 1.9)):
        assert not m.is_free(m.world_to_cell(x, y)), f'({x},{y}) 应该是墙'
    # 墙 B 在 y=2，x 从 9 到 18
    assert not m.is_free(m.world_to_cell(13.0, 2.0))
    # 墙 C 在 x=18，y 从 -9 到 -2
    assert not m.is_free(m.world_to_cell(18.0, -5.0))


def test_goal_marker_is_not_an_obstacle(world_map):
    """目标点标记只有 visual，不应该出现在地图里（否则车永远"到不了"）。"""
    _data, m = world_map
    assert m.is_free(m.world_to_cell(22.0, -6.0))


def test_spawn_is_free(world_map):
    _data, m = world_map
    assert m.is_free(m.world_to_cell(1.5, 1.5))


def test_astar_detours_around_wall(world_map):
    """目标在墙 A 后面：直线不可见，A* 必须向北绕。"""
    _data, m = world_map
    start, goal = (1.5, 1.5), (22.0, 1.5)
    assert not m.line_of_sight(m.world_to_cell(*start), m.world_to_cell(*goal))
    plan = m.inflated(0.28).plan(start, goal)
    assert plan is not None
    assert plan[-1] == pytest.approx(goal, abs=0.2)
    assert max(y for _x, y in plan) > 2.0
    assert len(plan) >= 3


def test_goal_inside_wall_snaps_to_nearest_free(world_map):
    """目标落在墙里时，A* 会把它挪到最近的空格，而不是直接报错。"""
    _data, m = world_map
    plan = m.inflated(0.28).plan((1.5, 1.5), (9.0, 0.0))
    assert plan is not None
    end = plan[-1]
    assert end != (9.0, 0.0)
    assert m.is_free(m.world_to_cell(*end))
