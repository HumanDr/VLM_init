"""LaserScan → 左/中/右扇区最近距离。

LiDAR 与深度相机是**互补**的：
  * 深度相机：有视场限制、受光照/纹理影响，但能看到三维形状（含地面）；
  * 激光雷达：360°，主动测距，不受光照影响，但只有一个扫描平面（0.21 m 高），
    探测不到低于扫描平面的小物体（比如很矮的台阶）。

两者都转成同一个 `SectorScan` 消息，控制器取**逐项最小值**，谁更保守听谁的。
"""
from __future__ import annotations

from typing import Dict

import numpy as np


def compute_scan_sectors(
    ranges,
    angle_min: float,
    angle_increment: float,
    range_min: float = 0.12,
    range_max: float = 20.0,
    front_half_angle: float = 0.35,   # 正前方 ±20°
    side_angle: float = 1.75,         # 左右各 100°
    max_range: float = 12.0,
) -> Dict[str, float]:
    """把一圈激光测距压成三个数字。

    Args:
        ranges: 每个角度的距离（m），无效值通常是 inf/nan
        front_half_angle: 正前方扇区的半角（rad）
        side_angle: 左右扇区的外边界（rad），超过这个角度不算"身边"
        max_range: 输出上限（比这更远的统一记成它）

    Returns dict: left, center, right, min_range, valid
    """
    r = np.asarray(ranges, dtype=np.float32)
    n = int(r.size)
    if n == 0:
        return {'left': max_range, 'center': max_range, 'right': max_range,
                'min_range': max_range, 'valid': False}

    lim = float(min(range_max, max_range))
    angles = angle_min + np.arange(n, dtype=np.float32) * float(angle_increment)
    # 把角度包到 [-pi, pi]，360° 雷达的 angle_min 通常是 -pi
    angles = (angles + np.pi) % (2.0 * np.pi) - np.pi

    valid = np.isfinite(r) & (r >= range_min) & (r <= lim)
    if not valid.any():
        return {'left': max_range, 'center': max_range, 'right': max_range,
                'min_range': max_range, 'valid': False}

    abs_a = np.abs(angles)
    front = valid & (abs_a <= front_half_angle)
    left = valid & (angles > front_half_angle) & (angles <= side_angle)
    right = valid & (angles < -front_half_angle) & (angles >= -side_angle)
    around = valid & (abs_a <= side_angle)

    def _min(mask: np.ndarray) -> float:
        return float(r[mask].min()) if mask.any() else max_range

    return {
        'left': _min(left),
        'center': _min(front),
        'right': _min(right),
        'min_range': _min(around),
        'valid': True,
    }
