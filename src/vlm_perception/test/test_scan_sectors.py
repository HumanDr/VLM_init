"""激光雷达扇区单测。"""
import math

import numpy as np
import pytest

from vlm_perception.scan_sectors import compute_scan_sectors

N = 720
INC = 2.0 * math.pi / N
AMIN = -math.pi


def make_scan(points: dict, default: float = 15.0):
    """points: {角度(rad): 距离(m)}；其余方向用 default（超出量程 = 无效）。"""
    r = np.full(N, default, np.float32)
    for ang, dist in points.items():
        i = int(round((ang - AMIN) / INC)) % N
        r[i] = dist
    return r


def test_front_obstacle():
    s = compute_scan_sectors(make_scan({0.0: 0.5}), AMIN, INC, max_range=12.0)
    assert s['valid'] is True
    assert s['center'] == pytest.approx(0.5, abs=0.02)
    assert s['left'] == pytest.approx(12.0)
    assert s['right'] == pytest.approx(12.0)


def test_left_and_right_obstacles():
    s = compute_scan_sectors(make_scan({0.6: 2.0, -1.0: 1.5}), AMIN, INC, max_range=12.0)
    assert s['left'] == pytest.approx(2.0, abs=0.02)
    assert s['right'] == pytest.approx(1.5, abs=0.02)
    assert s['center'] == pytest.approx(12.0)
    assert s['min_range'] == pytest.approx(1.5, abs=0.02)


def test_behind_is_ignored():
    """正后方（±180°）不属于左右扇区，不应影响判定。"""
    s = compute_scan_sectors(make_scan({math.pi - 0.01: 0.4}), AMIN, INC, max_range=12.0)
    assert s['left'] == pytest.approx(12.0)
    assert s['right'] == pytest.approx(12.0)
    assert s['min_range'] == pytest.approx(12.0)


def test_all_inf_is_invalid():
    s = compute_scan_sectors(np.full(N, np.inf, np.float32), AMIN, INC)
    assert s['valid'] is False
    assert s['center'] == pytest.approx(12.0)


def test_below_range_min_ignored():
    s = compute_scan_sectors(make_scan({0.0: 0.05}), AMIN, INC, range_min=0.12)
    assert s['center'] == pytest.approx(12.0)


def test_max_range_clips():
    s = compute_scan_sectors(make_scan({0.0: 30.0}), AMIN, INC,
                             range_max=20.0, max_range=12.0)
    assert s['center'] == pytest.approx(12.0)


def test_empty_scan():
    s = compute_scan_sectors(np.array([], np.float32), AMIN, INC)
    assert s['valid'] is False
