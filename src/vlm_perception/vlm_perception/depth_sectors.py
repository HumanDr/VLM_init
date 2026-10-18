"""深度图 → 左/中/右扇区最近距离。

作用：VLM 只有 1~2Hz，不能作为唯一的实时安全边界。这里的几何通道 15~20Hz，
提供一个廉价的"防撞兜底"，但它使用的是相机（深度相机），不引入激光雷达。

**地面过滤**：前置相机必然能看到脚下的地板，而地板永远比障碍物近。
如果直接对所有像素取分位数，会得到 ~0.5m 的假障碍，导致机器人永远不敢走
（这是实测踩到的坑）。所以按相机外参把每个像素反投影出高度，
把高度低于 `ground_clearance` 的点判为地面并剔除。
"""
from __future__ import annotations

import math
from typing import Dict, Tuple

import numpy as np


def compute_sectors(
    depth: np.ndarray,
    left_band: Tuple[float, float] = (0.00, 0.33),
    center_band: Tuple[float, float] = (0.33, 0.67),
    right_band: Tuple[float, float] = (0.67, 1.00),
    max_range: float = 5.0,
    percentile: float = 10.0,
    min_valid_ratio: float = 0.05,
    stride: int = 2,
    camera_height: float = 0.0,
    camera_pitch: float = 0.0,
    vfov: float = 0.0,
    ground_clearance: float = 0.06,
) -> Dict[str, float]:
    """按列切三个扇区，用低分位数估计"最近障碍物距离"。

    用低分位数而不是 min，是为了抗深度噪声/飞点；但分位数取 10% 而不是均值，
    保证依然对贴近的障碍物敏感。

    Args:
        camera_height: 相机光心距地高度 [m]；<=0 表示不做地面过滤
        camera_pitch:  相机光轴相对水平面下俯角 [rad]（正值向下）
        vfov:          垂直视场角 [rad]；地面过滤需要它
        ground_clearance: 反投影高度低于该值视为地面 [m]

    Returns dict with keys: left, center, right, min_range, valid
    """
    d = np.asarray(depth, dtype=np.float32)
    if d.ndim != 2 or d.size == 0:
        return {'left': max_range, 'center': max_range,
                'right': max_range, 'min_range': max_range, 'valid': False}

    h0, w0 = d.shape
    theta_row = None
    if camera_height > 0.0 and vfov > 0.0:
        # 每行光线相对水平面的俯角 beta = pitch + atan((v - cy) / fy)
        cy = (h0 - 1) / 2.0
        fy = (h0 / 2.0) / math.tan(vfov / 2.0)
        rows = np.arange(h0, dtype=np.float32).reshape(-1, 1)
        theta_row = np.arctan((rows - cy) / fy) + camera_pitch

    if stride > 1:
        d = d[::stride, ::stride]
        if theta_row is not None:
            theta_row = theta_row[::stride]

    # 无效值：NaN / Inf / 0 / 负 / 超量程
    valid_mask = np.isfinite(d) & (d > 0.05) & (d <= max_range)

    if theta_row is not None:
        # 点相对地面的高度：h - d*sin(beta)。低于 ground_clearance 的判为地面
        heights = camera_height - d * np.sin(theta_row)
        valid_mask &= heights > ground_clearance

    d = np.where(valid_mask, d, np.nan)

    h, w = d.shape
    bands = {
        'left': (int(left_band[0] * w), int(left_band[1] * w)),
        'center': (int(center_band[0] * w), int(center_band[1] * w)),
        'right': (int(right_band[0] * w), int(right_band[1] * w)),
    }

    out: Dict[str, float] = {}
    any_valid = False
    for name, (c0, c1) in bands.items():
        sub = d[:, max(0, c0):max(1, c1)]
        if sub.size == 0:
            out[name] = max_range
            continue
        vals = sub[np.isfinite(sub)]
        if vals.size == 0 or vals.size < min_valid_ratio * sub.size:
            out[name] = max_range
            continue
        out[name] = float(np.percentile(vals, percentile))
        any_valid = True

    out['min_range'] = float(min(out['left'], out['center'], out['right']))
    out['valid'] = bool(any_valid)
    return out
