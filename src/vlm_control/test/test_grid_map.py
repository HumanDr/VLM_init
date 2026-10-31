"""栅格地图 + A* 的单测（纯算法，不需要 ROS）。"""
import math

import pytest

from vlm_control.grid_map import GridMap


def grid(rows, res=1.0, origin=(0.0, 0.0)):
    return GridMap(rows, res, origin)


def test_coordinate_roundtrip():
    m = grid(["000", "000"], res=0.5, origin=(1.0, 2.0))
    c = m.world_to_cell(1.1, 2.9)
    assert c == (0, 1)
    x, y = m.cell_to_world(c)
    assert x == pytest.approx(1.25) and y == pytest.approx(2.75)


def test_straight_path_in_empty_map():
    m = grid(["00000", "00000", "00000"])
    path = m.plan((0.5, 0.5), (4.5, 0.5))
    assert path is not None
    assert path[0] == pytest.approx((0.5, 0.5))
    assert path[-1] == pytest.approx((4.5, 0.5))
    # 平滑后应该只剩起点和终点
    assert len(path) == 2


def test_detours_through_gap():
    # 第 2 行是墙，x=3 处留了一个缺口
    m = grid([
        "0000000",   # y=0
        "0000000",   # y=1
        "1110111",   # y=2 <- 墙
        "0000000",   # y=3
        "0000000",   # y=4
    ])
    path = m.plan((0.5, 0.5), (0.5, 4.5))
    assert path is not None
    assert max(y for _x, y in path) >= 2.0      # 必须走到墙上面
    # 顶点（平滑后的拐点）里应有一个落在缺口附近
    assert any(abs(x - 3.5) < 1.0 and abs(y - 2.5) < 1.0 for x, y in path)
    # 比直线长
    assert m.path_length(path) > math.dist((0.5, 0.5), (0.5, 4.5))


def test_no_path_when_wall_closed():
    m = grid(["0000000", "1111111", "0000000"])
    assert m.plan((0.5, 0.5), (0.5, 2.5)) is None


def test_start_inside_obstacle_uses_nearest_free():
    m = grid(["1100", "1100", "0000", "0000"])
    path = m.plan((0.5, 0.5), (3.5, 3.5))       # 起点在障碍里
    assert path is not None
    assert path[-1] == pytest.approx((3.5, 3.5))


def test_inflation_blocks_narrow_gap():
    # 缺口只有 1 格宽；膨胀 0.9 格后应该过不去
    m = grid(["0000000", "1110111", "0000000"], res=1.0)
    assert m.plan((0.5, 0.5), (0.5, 2.5)) is not None
    assert m.inflated(0.9).plan((0.5, 0.5), (0.5, 2.5)) is None


def test_no_corner_cutting():
    # 对角缝：两个障碍只在角上相接，斜着穿过去是不允许的
    m = grid([
        "000",
        "011",
        "001",
    ])
    # (0,2) -> (2,0) 的对角线会被两个障碍夹住
    assert m.line_of_sight((0, 2), (2, 0)) is False


def test_load_and_save_roundtrip(tmp_path):
    import json
    p = tmp_path / 'map.json'
    p.write_text(json.dumps({
        'resolution': 0.5, 'origin': [-1.0, -2.0],
        'rows': ['010', '000'],
    }), encoding='utf-8')
    m = GridMap.load(str(p))
    assert m.width == 3 and m.height == 2
    assert bool(m.grid[0][1]) is True
    assert m.is_free((0, 1)) is True
