"""真值里程计的坐标换算（纯函数）。"""
import math

import pytest

from vlm_control.ground_truth_odom import to_local, wrap_angle, yaw_from_quat


class _Q:
    def __init__(self, z=0.0, w=1.0, x=0.0, y=0.0):
        self.x, self.y, self.z, self.w = x, y, z, w


def _q_from_yaw(yaw):
    return _Q(z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def test_origin_offset():
    assert to_local(2.5, 1.5, 0.0, 1.5, 1.5, 0.0) == pytest.approx((1.0, 0.0, 0.0))


def test_origin_rotation():
    # 出生点朝 +y（90°），世界里的正前方 (0,1) 应该是 odom 的 (1,0)
    x, y, yaw = to_local(0.0, 1.0, math.pi / 2, 0.0, 0.0, math.pi / 2)
    assert (x, y) == pytest.approx((1.0, 0.0), abs=1e-9)
    assert yaw == pytest.approx(0.0, abs=1e-9)


def test_yaw_wraps():
    x, y, yaw = to_local(0.0, 0.0, -math.pi / 2, 0.0, 0.0, math.pi / 2)
    assert abs(yaw) == pytest.approx(math.pi, abs=1e-9)


def test_yaw_from_quat():
    for a in (0.0, 0.3, -1.2, 2.9):
        assert yaw_from_quat(_q_from_yaw(a)) == pytest.approx(wrap_angle(a), abs=1e-9)


def test_wrap_angle():
    assert abs(wrap_angle(3 * math.pi)) == pytest.approx(math.pi, abs=1e-9)
    assert wrap_angle(-0.1) == pytest.approx(-0.1)
