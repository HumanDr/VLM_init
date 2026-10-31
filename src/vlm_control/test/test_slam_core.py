"""SLAM 核心算法单测（纯函数，不需要 ROS/Gazebo）。"""
import math

import numpy as np
import pytest

from vlm_control.slam_core import (OccupancyMap, PoseGraph, SlamCore, average_poses,
                                   compose, inverse, relative, scan_points, wrap)


def test_compose_inverse_roundtrip():
    a = (1.0, 2.0, 0.7)
    b = (0.3, -0.5, -1.1)
    assert compose(a, inverse(a)) == pytest.approx((0.0, 0.0, 0.0), abs=1e-9)
    c = compose(a, b)
    assert compose(inverse(a), c) == pytest.approx(b, abs=1e-9)


def test_relative():
    a, b = (1.0, 1.0, 0.0), (2.0, 1.0, math.pi / 2)
    d = relative(a, b)
    assert d == pytest.approx((1.0, 0.0, math.pi / 2), abs=1e-9)


def test_average_poses_wraps_angles():
    x, y, th = average_poses([(0, 0, 3.1), (0, 0, -3.1)], [1, 1])
    assert x == pytest.approx(0) and y == pytest.approx(0)
    assert abs(wrap(th)) == pytest.approx(math.pi, abs=0.05)


def test_scan_points_filters_invalid():
    ranges = np.array([1.0, np.inf, 0.0, 20.0, 2.0])
    angles = np.zeros(5)
    pts = scan_points(ranges, angles, max_range=12.0, stride=1)
    assert len(pts) == 2          # 只有 1.0 和 2.0 有效


def test_occupancy_marks_wall_ahead():
    m = OccupancyMap(size_m=10.0, resolution=0.1, origin=(-5.0, -5.0))
    n = 36
    angles = np.linspace(-math.pi, math.pi, n, endpoint=False)
    m.update_scan((0.0, 0.0, 0.0), np.full(n, 2.0), angles, max_range=12.0)
    # 正前方 2m 处应被标记为占用
    col, row = m.to_cell(2.0, 0.0)
    assert m.log_odds[row, col] > 0.3
    # 正前方 0.5m 处应该被标成"空"
    col2, row2 = m.to_cell(0.5, 0.0)
    assert m.log_odds[row2, col2] <= 0.0


def test_pose_graph_reduces_loop_error():
    """航位推算带漂移、回环约束是"真实"的：优化后闭环误差应变小。"""
    a = 3.0
    truth = [(0.0, 0.0, 0.0)]
    for _ in range(4):
        truth.append(compose(truth[-1], (a, 0, math.pi / 2)))

    drifted = [truth[0]]
    for i in range(1, 5):
        d = relative(truth[i - 1], truth[i])
        drifted.append(compose(drifted[-1], (d[0] * 1.06, d[1] * 1.06, d[2] + 0.03)))

    g = PoseGraph()
    for p in drifted[1:]:
        g.add_node(p)
    for i in range(4):
        g.add_edge(i, i + 1, relative(drifted[i], drifted[i + 1]), 1.0)
    g.add_edge(4, 0, relative(truth[4], truth[0]), 1.0)   # 回环：真实相对位姿

    before = math.dist(g.poses[4][:2], g.poses[0][:2])
    g.optimize(iterations=80)
    after = math.dist(g.poses[4][:2], g.poses[0][:2])
    assert before > 0.2
    assert after < before * 0.5


def test_slam_core_first_scan_bootstraps():
    s = SlamCore(max_range=12.0)
    n = 72
    angles = np.linspace(-math.pi, math.pi, n, endpoint=False)
    r = s.process(np.full(n, 3.0), angles, wheel_delta=(0, 0, 0),
                  imu_delta=(0, 0, 0))
    assert r.pose == pytest.approx((0.0, 0.0, 0.0), abs=1e-6)
    assert r.score == 0.0            # 第一帧没有先验可匹配
    assert r.keyframes == 1


def test_slam_core_follows_wheel_prior_without_features():
    """雷达什么都看不到时，只能信先验，不能乱跳。"""
    s = SlamCore(max_range=12.0)
    n = 72
    angles = np.linspace(-math.pi, math.pi, n, endpoint=False)
    far = np.full(n, 20.0)                      # 全部超量程
    s.process(far, angles)
    r = s.process(far, angles, wheel_delta=(0.1, 0.0, 0.0))
    assert r.pose[0] == pytest.approx(0.1, abs=1e-6)
