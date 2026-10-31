"""多传感器 2D SLAM 的核心算法（纯 Python，无 ROS 依赖，可离线仿真验证）。

系统组成（对应 "相机 + IMU + 雷达" 三路传感器）：

    前端  雷达扫描匹配（似然场 + 由粗到精搜索）  <- 主定位
          IMU 陀螺积分（yaw 先验）
          轮式里程计（平移先验）
          RGB-D 视觉里程计（ORB + PnP，见 visual_odometry.py）  <- 额外约束
    后端  位姿图 + 回环检测 + Gauss-Seidel 迭代优化
    地图  对数几率占用栅格（射线投射，sliding 更新）

坐标系：`map` 系，原点 = 机器人启动位置，与项目里 `odom` 的约定一致。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

Pose = Tuple[float, float, float]        # (x, y, yaw)

# 占用栅格更新参数
L_FREE = -0.35
L_OCC = 0.85
L_MIN, L_MAX = -4.0, 4.0


# ---------------------------------------------------------------- SE(2)
def wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def compose(a: Pose, b: Pose) -> Pose:
    """a ⊗ b：先 a 再 b（b 在 a 的局部坐标系里）。"""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1],
            a[1] + s * b[0] + c * b[1],
            wrap(a[2] + b[2]))


def inverse(a: Pose) -> Pose:
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-(c * a[0] + s * a[1]), -(-s * a[0] + c * a[1]), wrap(-a[2]))


def relative(a: Pose, b: Pose) -> Pose:
    """从 a 到 b 的相对变换。"""
    return compose(inverse(a), b)


def average_poses(poses: Sequence[Pose], weights: Sequence[float]) -> Pose:
    w = np.asarray(weights, dtype=np.float64)
    w = w / max(1e-9, w.sum())
    x = sum(wi * p[0] for wi, p in zip(w, poses))
    y = sum(wi * p[1] for wi, p in zip(w, poses))
    th = math.atan2(sum(wi * math.sin(p[2]) for wi, p in zip(w, poses)),
                    sum(wi * math.cos(p[2]) for wi, p in zip(w, poses)))
    return (x, y, th)


# ---------------------------------------------------------------- 占用栅格
class OccupancyMap:
    """固定大小的对数几率栅格（默认 60m x 60m，0.05m/格）。"""

    def __init__(self, size_m: float = 60.0, resolution: float = 0.05,
                 origin: Tuple[float, float] = (-30.0, -30.0)) -> None:
        self.res = float(resolution)
        self.origin = (float(origin[0]), float(origin[1]))
        self.width = int(round(size_m / self.res))
        self.height = self.width
        self.log_odds = np.zeros((self.height, self.width), dtype=np.float32)

    # -- 坐标 --
    def to_cell(self, x: float, y: float) -> Tuple[int, int]:
        col = int(math.floor((x - self.origin[0]) / self.res))
        row = int(math.floor((y - self.origin[1]) / self.res))
        return col, row

    def inside(self, col: np.ndarray, row: np.ndarray) -> np.ndarray:
        return (col >= 0) & (col < self.width) & (row >= 0) & (row < self.height)

    # -- 更新 --
    def update_scan(self, pose: Pose, ranges: np.ndarray, angles: np.ndarray,
                    max_range: float = 12.0) -> None:
        """按一帧雷达更新：射线经过的格子记"空"，端点记"占用"。"""
        r = np.asarray(ranges, dtype=np.float32)
        a = np.asarray(angles, dtype=np.float32)
        valid = np.isfinite(r) & (r > 0.12) & (r <= max_range)
        if not valid.any():
            return
        r, a = r[valid], a[valid]

        # 雷达点 -> 世界坐标
        c, s = math.cos(pose[2]), math.sin(pose[2])
        wx = pose[0] + c * r * np.cos(a) - s * r * np.sin(a)
        wy = pose[1] + s * r * np.cos(a) + c * r * np.sin(a)

        # 端点：占用
        ec, er = self.to_cell_arr(wx, wy)
        m = self.inside(ec, er)
        np.add.at(self.log_odds, (er[m], ec[m]), L_OCC)

        # 射线采样：自由
        steps = np.maximum(1, np.round(r / 0.15)).astype(np.int32)
        max_steps = int(steps.max())
        for k in range(1, max_steps):
            t = np.clip(k / steps, 0.0, 1.0)
            sx = pose[0] + (wx - pose[0]) * t
            sy = pose[1] + (wy - pose[1]) * t
            sc, sr = self.to_cell_arr(sx, sy)
            m = self.inside(sc, sr) & (k < steps)
            np.add.at(self.log_odds, (sr[m], sc[m]), L_FREE)

        np.clip(self.log_odds, L_MIN, L_MAX, out=self.log_odds)

    def to_cell_arr(self, x: np.ndarray, y: np.ndarray):
        col = np.floor((x - self.origin[0]) / self.res).astype(np.int32)
        row = np.floor((y - self.origin[1]) / self.res).astype(np.int32)
        return col, row

    # -- 给规划器用 --
    def occupancy_int8(self, occ_thresh: float = 0.40,
                       free_thresh: float = -0.20) -> np.ndarray:
        out = np.full(self.log_odds.shape, -1, dtype=np.int8)
        out[self.log_odds > occ_thresh] = 100
        out[self.log_odds < free_thresh] = 0
        return out

    def probability(self) -> np.ndarray:
        return 1.0 - 1.0 / (1.0 + np.exp(self.log_odds))

    # -- 给匹配用 --
    def likelihood_patch(self, center: Pose, half_size_m: float = 6.0,
                         sigma: float = 0.25):
        """取以 center 为中心的一小块"命中似然"图。

        返回 (like, ox, oy)：like[r, c] 在 [0,1]，越靠近占用格子越大。
        """
        from scipy.ndimage import distance_transform_edt

        n = int(round(half_size_m / self.res))
        cx, cy = self.to_cell(center[0], center[1])
        c0, c1 = max(0, cx - n), min(self.width, cx + n)
        r0, r1 = max(0, cy - n), min(self.height, cy + n)
        if c1 - c0 < 4 or r1 - r0 < 4:
            return None
        occ = self.log_odds[r0:r1, c0:c1] > 0.25
        if not occ.any():
            return None
        dist = distance_transform_edt(~occ) * self.res
        like = np.exp(-(dist ** 2) / (2.0 * sigma ** 2)).astype(np.float32)
        ox = self.origin[0] + c0 * self.res
        oy = self.origin[1] + r0 * self.res
        return like, ox, oy


# ---------------------------------------------------------------- 扫描匹配
@dataclass
class MatchResult:
    pose: Pose
    score: float
    ok: bool


def scan_points(ranges: np.ndarray, angles: np.ndarray,
                max_range: float = 12.0, stride: int = 4) -> np.ndarray:
    r = np.asarray(ranges, dtype=np.float64)[::stride]
    a = np.asarray(angles, dtype=np.float64)[::stride]
    valid = np.isfinite(r) & (r > 0.12) & (r <= max_range)
    r, a = r[valid], a[valid]
    return np.stack([r * np.cos(a), r * np.sin(a)], axis=1)


def match_scan(like: np.ndarray, ox: float, oy: float, res: float,
               pts: np.ndarray, prior: Pose,
               levels: Sequence[Tuple[float, float, int, int]] = (
                   (0.10, 0.050, 6, 3),
                   (0.04, 0.020, 5, 3)),
               min_score: float = 0.25) -> MatchResult:
    """似然场上的由粗到精搜索（穷举平移 + yaw，向量化）。

    levels: (平移步长 m, yaw 步长 rad, 平移半径格数, yaw 半径格数)
    """
    if like is None or pts.shape[0] < 10:
        return MatchResult(prior, 0.0, False)
    h, w = like.shape
    best = prior
    best_score = -1.0

    for (step_xy, step_th, n_xy, n_th) in levels:
        dxs = np.arange(-n_xy, n_xy + 1) * step_xy
        dys = np.arange(-n_xy, n_xy + 1) * step_xy
        offs = np.stack(np.meshgrid(dxs, dys, indexing='ij'), axis=-1).reshape(-1, 2)
        for k in range(-n_th, n_th + 1):
            th = best[2] + k * step_th
            c, s = math.cos(th), math.sin(th)
            rot = pts @ np.array([[c, s], [-s, c]])
            # (N, P, 2)
            world = rot[None, :, :] + offs[:, None, :]
            col = np.floor((world[..., 0] - ox) / res).astype(np.int32)
            row = np.floor((world[..., 1] - oy) / res).astype(np.int32)
            valid = (col >= 0) & (col < w) & (row >= 0) & (row < h)
            counts = valid.sum(axis=1)
            if not valid.any():
                continue
            total = np.where(valid, like[np.clip(row, 0, h - 1),
                                        np.clip(col, 0, w - 1)], 0.0).sum(axis=1)
            # 只对落在窗口里的点求平均；覆盖率太低（<40%）的候选直接作废
            score = np.where(counts >= max(10, 0.4 * pts.shape[0]),
                             total / np.maximum(1, counts), 0.0)
            idx = int(np.argmax(score))
            if score[idx] > best_score:
                best_score = float(score[idx])
                best = (best[0] + offs[idx, 0], best[1] + offs[idx, 1], wrap(th))
    return MatchResult(best, best_score, best_score >= min_score)


# ---------------------------------------------------------------- 位姿图
@dataclass
class Edge:
    i: int
    j: int
    delta: Pose          # pose_j = pose_i ⊗ delta
    weight: float


@dataclass
class PoseGraph:
    poses: List[Pose] = field(default_factory=lambda: [(0.0, 0.0, 0.0)])
    edges: List[Edge] = field(default_factory=list)

    def add_node(self, pose: Pose) -> int:
        self.poses.append(pose)
        return len(self.poses) - 1

    def add_edge(self, i: int, j: int, delta: Pose, weight: float = 1.0) -> None:
        if i == j:
            return
        self.edges.append(Edge(i, j, delta, float(weight)))

    def optimize(self, iterations: int = 25) -> None:
        """Gauss-Seidel 松弛：每个节点取所有相邻约束预测的加权平均。"""
        if len(self.poses) <= 2 or not self.edges:
            return
        adj: Dict[int, List[int]] = {i: [] for i in range(len(self.poses))}
        for e_i, e in enumerate(self.edges):
            adj[e.i].append(e_i)
            adj[e.j].append(e_i)

        for _ in range(iterations):
            for k in range(1, len(self.poses)):
                preds, ws = [], []
                for ei in adj.get(k, []):
                    e = self.edges[ei]
                    if e.j == k:
                        preds.append(compose(self.poses[e.i], e.delta))
                    else:
                        preds.append(compose(self.poses[e.j], inverse(e.delta)))
                    ws.append(e.weight)
                if preds:
                    self.poses[k] = average_poses(preds, ws)


def scan_likelihood(pts_world: np.ndarray, center: Pose,
                    half_size_m: float = 5.0, res: float = 0.05,
                    sigma: float = 0.15):
    """把一帧扫描点画成局部"命中似然"图（用于扫描对扫描匹配）。"""
    from scipy.ndimage import distance_transform_edt

    n = int(round(half_size_m / res))
    ox = center[0] - n * res
    oy = center[1] - n * res
    occ = np.zeros((2 * n, 2 * n), dtype=bool)
    col = np.floor((pts_world[:, 0] - ox) / res).astype(np.int32)
    row = np.floor((pts_world[:, 1] - oy) / res).astype(np.int32)
    ok = (col >= 0) & (col < occ.shape[1]) & (row >= 0) & (row < occ.shape[0])
    if ok.sum() < 10:
        return None
    occ[row[ok], col[ok]] = True
    dist = distance_transform_edt(~occ) * res
    like = np.exp(-(dist ** 2) / (2.0 * sigma ** 2)).astype(np.float32)
    return like, ox, oy


# ---------------------------------------------------------------- 顶层
@dataclass
class SlamResult:
    pose: Pose
    score: float
    keyframes: int
    loop_closed: bool


class SlamCore:
    """把"雷达 + 先验"变成"修正后的位姿 + 地图"。

    调用顺序：每来一帧雷达就 `process()` 一次；
    `wheel_delta` / `imu_delta` / `vo_delta` 是"自上一帧以来"的相对运动先验。
    """

    def __init__(self, resolution: float = 0.05, max_range: float = 12.0,
                 keyframe_dist: float = 0.35, keyframe_angle: float = 0.20,
                 loop_radius: float = 3.0, loop_exclude: int = 8,
                 vo_weight: float = 0.35) -> None:
        self.map = OccupancyMap(resolution=resolution)
        self.max_range = max_range
        self.keyframe_dist = keyframe_dist
        self.keyframe_angle = keyframe_angle
        self.loop_radius = loop_radius
        self.loop_exclude = loop_exclude
        self.vo_weight = vo_weight

        self.graph = PoseGraph()
        self.pose: Pose = (0.0, 0.0, 0.0)
        self._keyframe_scans: List[Tuple[Pose, np.ndarray, np.ndarray]] = []
        self._last_keyframe = 0
        self._last_scan: Optional[Tuple[np.ndarray, np.ndarray]] = None
        self._prev_points: Optional[np.ndarray] = None

    # ------------------------------------------------------------ 主循环
    def process(self, ranges: np.ndarray, angles: np.ndarray,
                wheel_delta: Optional[Pose] = None,
                imu_delta: Optional[Pose] = None,
                vo_delta: Optional[Pose] = None,
                ) -> SlamResult:
        prior = compose(self.pose, wheel_delta or (0.0, 0.0, 0.0))
        # yaw 先验：IMU 更可信，VO 作为补充
        yaw_delta = (imu_delta[2] if imu_delta else 0.0)
        if vo_delta is not None:
            yaw_delta = 0.6 * yaw_delta + 0.4 * vo_delta[2]
        prior = (prior[0], prior[1], wrap(self.pose[2] + yaw_delta))

        stride = max(1, len(ranges) // 180)      # 720 线取 180 点，180 线取 90 点
        pts = scan_points(ranges, angles, self.max_range, stride=stride)
        score = 0.0
        corrected = prior

        # 扫描对扫描（用上一帧的局部似然场）：不受全局地图"被漂移带歪"的影响
        if self._prev_points is not None:
            field = scan_likelihood(self._prev_points, self.pose,
                                    half_size_m=6.0, res=self.map.res)
            if field is not None:
                like, ox, oy = field
                # 搜索窗口跟"预期运动量"挂钩：几乎不动时收紧，防止沿墙打滑
                d = math.dist(prior[:2], self.pose[:2])
                dth = abs(wrap(prior[2] - self.pose[2]))
                half_xy = max(0.12, min(0.30, 3.0 * d + 0.06))
                half_th = max(0.04, min(0.15, 3.0 * dth + 0.02))
                n_xy = 3
                levels = ((half_xy / n_xy, half_th / 2, n_xy, 2),
                          (half_xy / (3 * n_xy), half_th / 5, 2, 2))
                res = match_scan(like, ox, oy, self.map.res, pts, prior,
                                 levels=levels, min_score=0.55)
                score = res.score
                if res.ok:
                    dx = res.pose[0] - prior[0]
                    dy = res.pose[1] - prior[1]
                    if math.hypot(dx, dy) <= 0.35 and abs(wrap(res.pose[2] - prior[2])) <= 0.12:
                        corrected = res.pose

        self.pose = corrected
        # 当前帧的扫描点（世界系，供下一帧做扫描对扫描）
        c_, s_ = math.cos(corrected[2]), math.sin(corrected[2])
        self._prev_points = pts @ np.array([[c_, s_], [-s_, c_]]) + np.array(
            [corrected[0], corrected[1]])
        self.map.update_scan(corrected, ranges, angles, self.max_range)

        loop_closed = self._maybe_keyframe(ranges, angles, score, vo_delta, imu_delta)
        return SlamResult(self.pose, score, len(self.graph.poses), loop_closed)

    # ------------------------------------------------------------ 关键帧/回环
    def _maybe_keyframe(self, ranges, angles, score, vo_delta, imu_delta) -> bool:
        moved = math.dist(self.pose[:2], self.graph.poses[-1][:2])
        turned = abs(wrap(self.pose[2] - self.graph.poses[-1][2]))
        if moved < self.keyframe_dist and turned < self.keyframe_angle:
            return False

        prev = self.graph.poses[-1]
        k = self.graph.add_node(self.pose)
        self.graph.add_edge(k - 1, k, relative(prev, self.pose), 1.0)
        if vo_delta is not None:
            self.graph.add_edge(k - 1, k, vo_delta, self.vo_weight)

        self._keyframe_scans.append((self.pose, np.array(ranges, dtype=np.float32),
                                     np.array(angles, dtype=np.float32)))

        # ---- 回环检测：找附近的历史关键帧，用它的扫描做一次相对匹配 ----
        closed = False
        best = self._find_loop_candidate()
        if best is not None:
            other_pose, other_ranges, other_angles = best
            rel_guess = relative(other_pose, self.pose)
            if self._verify_loop(other_pose, other_ranges, other_angles,
                                 rel_guess, ranges, angles):
                idx = self._keyframe_index(best)
                self.graph.add_edge(idx, k, rel_guess, 2.0)
                self.graph.optimize(iterations=30)
                self.pose = self.graph.poses[k]
                self._rebuild_map()
                closed = True
        return closed

    def _keyframe_index(self, item) -> int:
        for i, kf in enumerate(self._keyframe_scans):
            if kf[0] == item[0]:
                return i
        return 0

    def _find_loop_candidate(self):
        if len(self._keyframe_scans) <= self.loop_exclude + 1:
            return None
        best = None
        best_d = self.loop_radius
        for kf in self._keyframe_scans[:-self.loop_exclude]:
            d = math.dist(kf[0][:2], self.pose[:2])
            if d < best_d:
                best, best_d = kf, d
        return best

    def _verify_loop(self, other_pose, other_ranges, other_angles,
                     rel_guess, ranges, angles) -> bool:
        tmp = OccupancyMap(size_m=16.0, resolution=self.map.res,
                           origin=(other_pose[0] - 8.0, other_pose[1] - 8.0))
        tmp.update_scan(other_pose, other_ranges, other_angles, self.max_range)
        patch = tmp.likelihood_patch(self.pose, half_size_m=6.0)
        if patch is None:
            return False
        like, ox, oy = patch
        stride = max(1, len(ranges) // 180)      # 720 线取 180 点，180 线取 90 点
        pts = scan_points(ranges, angles, self.max_range, stride=stride)
        res = match_scan(like, ox, oy, tmp.res, pts, self.pose,
                         levels=((0.08, 0.04, 5, 3), (0.03, 0.015, 4, 2)),
                         min_score=0.35)
        if not res.ok:
            return False
        return (math.dist(res.pose[:2], self.pose[:2]) < 2.5
                and abs(wrap(res.pose[2] - self.pose[2])) < 0.6)

    # ------------------------------------------------------------ 重建地图
    def _rebuild_map(self) -> None:
        self.map.log_odds[:] = 0.0
        for pose, ranges, angles in self._keyframe_scans:
            self.map.update_scan(pose, ranges, angles, self.max_range)
