#!/usr/bin/env python3
"""离线 SLAM 仿真：不需要 Gazebo，验证"漂移能否被修回来"。

做法：
  1. 把 warehouse_map.json 当成真实世界，射线投射生成**完美的激光雷达**；
  2. 让机器人沿一条闭合路线走一圈（这样才有回环）；
  3. 给轮式里程计人为加漂移（尺度 + yaw 偏置），IMU 只加噪声；
  4. 把扫描 + 漂移先验喂给 SlamCore，比较它的位姿和真值的误差。

    python3 scripts/slam_sim.py                 # 默认一圈 ~20m
    python3 scripts/slam_sim.py --laps 2        # 两圈，回环更明显

它同时是算法回归工具：CI 里跑一次，看"漂移 vs SLAM 误差"这两个数。
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time
from typing import List, Tuple

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src', 'vlm_control'))

from vlm_control.grid_map import GridMap          # noqa: E402
from vlm_control.slam_core import SlamCore, compose, relative  # noqa: E402

Pose = Tuple[float, float, float]


def wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class RayCaster:
    """在栅格地图上做射线投射，模拟激光雷达。"""

    def __init__(self, grid: GridMap, max_range: float = 12.0,
                 n_rays: int = 180, noise: float = 0.01,
                 outlier_p: float = 0.03, seed: int = 0) -> None:
        self.g = grid
        self._grid = np.array(grid.grid, dtype=bool)
        self.max_range = max_range
        self.n_rays = n_rays
        self.noise = noise
        self.outlier_p = outlier_p
        self.rng = np.random.default_rng(seed)
        self.angles = np.linspace(-math.pi, math.pi, n_rays, endpoint=False)

    def cast(self, pose: Pose) -> np.ndarray:
        step = self.g.resolution
        n_steps = int(self.max_range / step)
        r = np.full(self.n_rays, self.max_range, dtype=np.float64)
        c, s = math.cos(pose[2]), math.sin(pose[2])
        cos_a = np.cos(self.angles)
        sin_a = np.sin(self.angles)
        dx = (c * cos_a - s * sin_a) * step
        dy = (s * cos_a + c * sin_a) * step
        hit = np.zeros(self.n_rays, dtype=bool)
        for k in range(1, n_steps + 1):
            x = pose[0] + dx * k
            y = pose[1] + dy * k
            col = np.floor((x - self.g.origin[0]) / step).astype(np.int32)
            row = np.floor((y - self.g.origin[1]) / step).astype(np.int32)
            inside = ((col >= 0) & (col < self.g.width)
                      & (row >= 0) & (row < self.g.height))
            occ = np.zeros(self.n_rays, dtype=bool)
            if inside.any():
                occ[inside] = self._grid[row[inside], col[inside]]
            new_hit = occ & ~hit
            r[new_hit] = k * step
            hit |= new_hit
            if hit.all():
                break
        # 噪声 + 少量离群值
        r = r + self.rng.normal(0.0, self.noise, size=r.shape)
        out = self.rng.random(self.n_rays) < self.outlier_p
        r[out] = self.rng.uniform(0.2, self.max_range, size=int(out.sum()))
        return np.clip(r, 0.12, self.max_range)


def make_trajectory(laps: int = 1, speed: float = 0.45, dt: float = 0.1) -> List[Pose]:
    """一条有拐弯、能闭合的路线（在仓库西侧房间里绕一圈）。"""
    waypoints = [(1.5, 1.5), (1.5, 6.0), (6.0, 6.0), (6.0, 1.5), (1.5, 1.5)]
    pts = waypoints * laps
    poses: List[Pose] = []
    x, y = pts[0]
    yaw = 0.0
    step = speed * dt
    for i in range(len(pts) - 1):
        tx, ty = pts[i + 1]
        cx, cy = x, y
        # 先原地转向目标
        target_yaw = math.atan2(ty - cy, tx - cx)
        n_turn = max(1, int(abs(wrap(target_yaw - yaw)) / (1.2 * dt)))
        for _ in range(n_turn):
            yaw = wrap(yaw + wrap(target_yaw - yaw) / max(1, n_turn - 0))
            poses.append((x, y, yaw))
        dist = math.hypot(tx - cx, ty - cy)
        n = max(1, int(dist / step))
        for k in range(n):
            x = cx + (tx - cx) * (k + 1) / n
            y = cy + (ty - cy) * (k + 1) / n
            poses.append((x, y, yaw))
    return poses


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--laps', type=int, default=1)
    ap.add_argument('--map', default=os.path.join(
        ROOT, 'src/vlm_avoid_bringup/config/warehouse_map.json'))
    ap.add_argument('--yaw-bias', type=float, default=0.0016,
                    help='轮式里程计每帧的 yaw 偏置 (rad)')
    ap.add_argument('--scale', type=float, default=1.05,
                    help='轮式里程计平移尺度误差')
    ap.add_argument('--imu-noise', type=float, default=0.004)
    ap.add_argument('--seed', type=int, default=0)
    args = ap.parse_args()

    grid = GridMap.load(args.map)
    caster = RayCaster(grid, noise=0.01, seed=args.seed)
    traj = make_trajectory(args.laps)
    rng = np.random.default_rng(args.seed + 1)

    slam = SlamCore(max_range=12.0, keyframe_dist=0.4, keyframe_angle=0.25,
                    loop_radius=1.5, loop_exclude=8)
    wheel: Pose = (0.0, 0.0, 0.0)      # 漂移中的轮式里程计
    prev_truth = traj[0]

    err_drift, err_slam = [], []
    t0 = time.time()
    for i, truth in enumerate(traj):
        ranges = caster.cast(truth)
        true_delta = relative(prev_truth, truth)
        # 轮式里程计：尺度误差 + yaw 偏置 + 噪声
        wheel_delta = (true_delta[0] * args.scale + rng.normal(0, 0.002),
                       true_delta[1] * args.scale + rng.normal(0, 0.002),
                       true_delta[2] * args.scale + args.yaw_bias + rng.normal(0, 0.001))
        imu_delta = (0.0, 0.0, true_delta[2] + rng.normal(0, args.imu_noise))
        wheel = compose(wheel, wheel_delta)

        res = slam.process(ranges, caster.angles, wheel_delta=wheel_delta,
                           imu_delta=imu_delta)
        prev_truth = truth

        # SLAM 的坐标系原点 = 起点，真值也要换算到同一个坐标系才能比较
        truth_local = relative(traj[0], truth)
        err_drift.append(math.dist(wheel[:2], truth_local[:2]))
        err_slam.append(math.dist(res.pose[:2], truth_local[:2]))

    dt = time.time() - t0
    final_drift, final_slam = err_drift[-1], err_slam[-1]
    max_drift, max_slam = max(err_drift), max(err_slam)
    print(f'帧数 {len(traj)}，用时 {dt:.1f}s（{len(traj)/max(dt,1e-9):.0f} 帧/s）')
    print(f'路线长度         {sum(math.dist(traj[i][:2], traj[i+1][:2]) for i in range(len(traj)-1)):.1f} m')
    print(f'轮式里程计漂移   最终 {final_drift:.2f} m / 最大 {max_drift:.2f} m')
    print(f'SLAM 误差        最终 {final_slam:.2f} m / 最大 {max_slam:.2f} m')
    print(f'关键帧 {len(slam.graph.poses)}，位姿图边 {len(slam.graph.edges)}')
    occ = slam.map.occupancy_int8()
    print(f'地图占用格子     {int((occ == 100).sum())}（{(occ == 100).mean()*100:.2f}%）')

    ok = final_slam < 0.4 and final_slam < final_drift
    print('结论：' + ('✓ SLAM 把漂移压回 0.4 m 内' if ok
                   else '✗ 误差偏大，需要调参'))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
