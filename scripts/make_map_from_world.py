#!/usr/bin/env python3
"""从 Gazebo 的 .world（SDF）生成 2D 占用栅格地图（JSON），给 A* 用。

    python3 scripts/make_map_from_world.py                       # 默认 warehouse
    python3 scripts/make_map_from_world.py --world src/vlbot_gazebo/worlds/corridor.world \
        --out src/vlm_avoid_bringup/config/corridor_map.json

只把"静态模型 + 碰撞体"画进地图：
  * box / cylinder / sphere 会栅格化；
  * plane（地面）跳过；
  * 只有 visual 没有 collision 的模型（比如目标点标记）不会出现在地图里——
    这和激光雷达/深度相机看到的一致。
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np


def _parse_pose(elem) -> tuple:
    pose = elem.find('pose')
    if pose is None or not (pose.text or '').strip():
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    vals = [float(v) for v in pose.text.split()]
    while len(vals) < 6:
        vals.append(0.0)
    return tuple(vals[:6])


def _collision_shapes(model: ET.Element):
    """返回 (rects, circles)，坐标已是世界系；rect 为 (cx, cy, sx, sy)。"""
    rects, circles = [], []
    mx, my, _mz, _mr, _mp, myaw = _parse_pose(model)
    for link in model.findall('link'):
        for col in link.findall('collision'):
            cx, cy, _cz, _cr, _cp, cyaw = _parse_pose(col)
            # 先按 yaw 旋转到模型坐标系，再加模型位姿（本项目世界都是 0°，这里保守处理）
            a = myaw
            gx = mx + cx * math.cos(a) - cy * math.sin(a)
            gy = my + cx * math.sin(a) + cy * math.cos(a)
            geom = col.find('geometry')
            if geom is None:
                continue
            box = geom.find('box')
            cyl = geom.find('cylinder')
            sph = geom.find('sphere')
            if box is not None:
                size = box.find('size')
                if size is None:
                    continue
                sx, sy, _sz = [float(v) for v in size.text.split()]
                yaw = a + cyaw
                if abs(yaw) > 1e-6:
                    # 旋转矩形 -> 保守地用它的外接 AABB
                    c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
                    sx, sy = sx * c + sy * s, sx * s + sy * c
                rects.append((gx, gy, sx, sy))
            elif cyl is not None:
                r = float(cyl.find('radius').text)
                circles.append((gx, gy, r))
            elif sph is not None:
                r = float(sph.find('radius').text)
                circles.append((gx, gy, r))
    return rects, circles


def build(world_path: str, resolution: float = 0.1, margin: float = 0.6,
          include: tuple = ()) -> dict:
    root = ET.parse(world_path).getroot()
    world = root.find('.//world')
    if world is None:
        raise SystemExit(f'{world_path} 里没有 <world>')

    shapes = []
    for model in world.findall('model'):
        name = model.get('name', '?')
        if include and name not in include:
            continue
        static = model.find('static')
        if static is None or (static.text or '').strip().lower() not in ('1', 'true'):
            continue
        rects, circles = _collision_shapes(model)
        for r in rects:
            shapes.append(('rect', r, name))
        for c in circles:
            shapes.append(('circle', c, name))

    if not shapes:
        raise SystemExit('没有找到任何静态碰撞体')

    xs_min = min(r[0] - r[2] / 2 for t, r, _ in shapes if t == 'rect') if any(
        t == 'rect' for t, _, _ in shapes) else 1e9
    xs_max = max(r[0] + r[2] / 2 for t, r, _ in shapes if t == 'rect') if any(
        t == 'rect' for t, _, _ in shapes) else -1e9
    ys_min = min(r[1] - r[3] / 2 for t, r, _ in shapes if t == 'rect') if any(
        t == 'rect' for t, _, _ in shapes) else 1e9
    ys_max = max(r[1] + r[3] / 2 for t, r, _ in shapes if t == 'rect') if any(
        t == 'rect' for t, _, _ in shapes) else -1e9
    for t, c, _ in shapes:
        if t == 'circle':
            xs_min = min(xs_min, c[0] - c[2])
            xs_max = max(xs_max, c[0] + c[2])
            ys_min = min(ys_min, c[1] - c[2])
            ys_max = max(ys_max, c[1] + c[2])

    ox, oy = xs_min - margin, ys_min - margin
    w = int(math.ceil((xs_max + margin - ox) / resolution))
    h = int(math.ceil((ys_max + margin - oy) / resolution))

    cx = ox + (np.arange(w) + 0.5) * resolution
    cy = oy + (np.arange(h) + 0.5) * resolution
    X, Y = np.meshgrid(cx, cy)          # 行索引对应 y（0 = 最下面）

    occ = np.zeros((h, w), dtype=bool)
    for t, geom, _name in shapes:
        if t == 'rect':
            gx, gy, sx, sy = geom
            occ |= (np.abs(X - gx) <= sx / 2) & (np.abs(Y - gy) <= sy / 2)
        else:
            gx, gy, r = geom
            occ |= (X - gx) ** 2 + (Y - gy) ** 2 <= r * r

    rows = [''.join('1' if v else '0' for v in occ[y]) for y in range(h)]
    return {
        'resolution': resolution,
        'origin': [round(ox, 4), round(oy, 4)],
        'width': w,
        'height': h,
        'world': os.path.basename(world_path),
        'occupied_cells': int(occ.sum()),
        'rows': rows,
    }


def main() -> int:
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ap = argparse.ArgumentParser()
    ap.add_argument('--world', default=os.path.join(
        here, 'src/vlbot_gazebo/worlds/warehouse.world'))
    ap.add_argument('--out', default=os.path.join(
        here, 'src/vlm_avoid_bringup/config/warehouse_map.json'))
    ap.add_argument('--resolution', type=float, default=0.1)
    args = ap.parse_args()

    data = build(args.world, args.resolution)
    with open(args.out, 'w', encoding='utf-8') as f:
        json.dump(data, f, ensure_ascii=False, separators=(',', ':'))
        f.write('\n')
    print(f'✓ {args.out}')
    print(f'  {data["width"]}x{data["height"]} 格 @ {data["resolution"]}m, '
          f'origin={data["origin"]}, 占用 {data["occupied_cells"]} 格 '
          f'({100.0 * data["occupied_cells"] / (data["width"] * data["height"]):.1f}%)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
