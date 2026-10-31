"""栅格地图 + A* 路径规划（纯 Python，无 ROS 依赖，可离线单测）。

地图文件格式（JSON，仓库里由 scripts/make_map_from_world.py 从 SDF 世界生成）::

    {
      "resolution": 0.1,          # 每格多少米
      "origin": [min_x, min_y],   # 左下角的世界坐标
      "rows": [                   # rows[0] 对应 y = min_y（**从下往上**）
        "0000001111000000",
        ...
      ]
    }

`1` 表示占用（墙/障碍），`0` 表示可走。

A* 相关约定：
  * 8 邻域，直走代价 1、斜走 √2；
  * 禁止"切角"（斜着穿过两个障碍的接缝）；
  * 起点/终点落在障碍里时，先就近找最近的空格（机器人贴墙时会出现）。
"""
from __future__ import annotations

import heapq
import json
import math
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

Cell = Tuple[int, int]      # (col, row)
Point = Tuple[float, float]

_DIRS = ((1, 0), (-1, 0), (0, 1), (0, -1),
         (1, 1), (1, -1), (-1, 1), (-1, -1))


class GridMap:
    def __init__(self, rows: Sequence[str], resolution: float,
                 origin: Sequence[float], occupied_char: str = '1') -> None:
        if not rows:
            raise ValueError('地图为空')
        w = len(rows[0])
        if any(len(r) != w for r in rows):
            raise ValueError('地图每行长度必须一致')
        self.rows = list(rows)
        self.resolution = float(resolution)
        self.origin = (float(origin[0]), float(origin[1]))
        self.width = w
        self.height = len(rows)
        self.grid = np.array([[c == occupied_char for c in row] for row in rows],
                             dtype=bool)

    # ------------------------------------------------------------ 构造
    @classmethod
    def load(cls, path: str) -> 'GridMap':
        with open(path, encoding='utf-8') as f:
            data = json.load(f)
        return cls(data['rows'], data['resolution'], data['origin'])

    @classmethod
    def from_array(cls, occ: np.ndarray, resolution: float,
                   origin: Sequence[float]) -> 'GridMap':
        """从布尔数组直接构造（在线 SLAM 的 /map 用，避免字符串来回转）。"""
        obj = cls.__new__(cls)
        obj.rows = None
        obj.resolution = float(resolution)
        obj.origin = (float(origin[0]), float(origin[1]))
        obj.grid = np.asarray(occ, dtype=bool)
        obj.height, obj.width = obj.grid.shape
        return obj

    def clone(self) -> 'GridMap':
        return GridMap(self.rows, self.resolution, self.origin)

    # ------------------------------------------------------------ 坐标
    def world_to_cell(self, x: float, y: float) -> Cell:
        ox, oy = self.origin
        return (int(math.floor((x - ox) / self.resolution)),
                int(math.floor((y - oy) / self.resolution)))

    def cell_to_world(self, c: Cell) -> Point:
        ox, oy = self.origin
        return (ox + (c[0] + 0.5) * self.resolution,
                oy + (c[1] + 0.5) * self.resolution)

    def in_bounds(self, c: Cell) -> bool:
        return 0 <= c[0] < self.width and 0 <= c[1] < self.height

    def is_free(self, c: Cell) -> bool:
        return self.in_bounds(c) and not self.grid[c[1]][c[0]]

    def set_occupied(self, c: Cell) -> None:
        if self.in_bounds(c):
            self.grid[c[1]][c[0]] = True

    # ------------------------------------------------------------ 膨胀
    def inflated(self, radius_m: float) -> 'GridMap':
        """把障碍向外膨胀 radius_m（机器人半径 + 安全余量）。

        优先用 scipy 的欧氏距离变换（O(n)，1200x1200 约 40ms）；
        没有 scipy 时退回逐个障碍点的圆邻域。
        """
        r = int(math.ceil(max(0.0, radius_m) / self.resolution))
        if r <= 0:
            return self.clone()
        try:
            from scipy.ndimage import distance_transform_edt
            if self.grid.any():
                dist = distance_transform_edt(~self.grid) * self.resolution
                # 距离是到格子中心的；再补半格，等价于"到障碍边缘"，偏保守更安全
                out = GridMap.from_array(dist <= radius_m + 0.5 * self.resolution,
                                         self.resolution, self.origin)
                return out
        except ImportError:
            pass
        out = self.clone()
        obstacles = np.argwhere(self.grid)
        for (y, x) in obstacles:
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    if dx * dx + dy * dy > r * r:
                        continue
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < self.width and 0 <= ny < self.height:
                        out.grid[ny, nx] = True
        return out

    # ------------------------------------------------------------ 搜索
    def nearest_free(self, c: Cell, max_radius_cells: int = 30) -> Optional[Cell]:
        """就近找空格（起点/终点落在障碍/膨胀区里时用）。"""
        if self.is_free(c):
            return c
        for r in range(1, max_radius_cells + 1):
            best = None
            best_d = 1e9
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    if max(abs(dx), abs(dy)) != r:
                        continue
                    nc = (c[0] + dx, c[1] + dy)
                    if self.is_free(nc):
                        d = dx * dx + dy * dy
                        if d < best_d:
                            best, best_d = nc, d
            if best is not None:
                return best
        return None

    def line_of_sight(self, a: Cell, b: Cell) -> bool:
        """两个格子之间是否能直线通行（Bresenham，含端点）。"""
        x0, y0 = a
        x1, y1 = b
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        while True:
            if not self.is_free((x0, y0)):
                return False
            if x0 == x1 and y0 == y1:
                return True
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

    def astar(self, start: Cell, goal: Cell,
              max_expand: int = 400000) -> Optional[List[Cell]]:
        """A*：返回从 start 到 goal 的格子序列（含两端），无解返回 None。"""
        start = self.nearest_free(start)
        goal = self.nearest_free(goal)
        if start is None or goal is None:
            return None
        if start == goal:
            return [start]

        def h(c: Cell) -> float:
            dx = abs(c[0] - goal[0])
            dy = abs(c[1] - goal[1])
            return (dx + dy) + (math.sqrt(2) - 2) * min(dx, dy)   # octile

        open_heap: List[Tuple[float, float, Cell]] = [(h(start), 0.0, start)]
        came: Dict[Cell, Cell] = {}
        g: Dict[Cell, float] = {start: 0.0}
        closed = set()
        expanded = 0

        while open_heap:
            _, gc, cur = heapq.heappop(open_heap)
            if cur in closed:
                continue
            if cur == goal:
                path = [cur]
                while cur in came:
                    cur = came[cur]
                    path.append(cur)
                path.reverse()
                return path
            closed.add(cur)
            expanded += 1
            if expanded > max_expand:
                return None
            for dx, dy in _DIRS:
                nxt = (cur[0] + dx, cur[1] + dy)
                if not self.is_free(nxt) or nxt in closed:
                    continue
                if dx != 0 and dy != 0:
                    # 禁止切角：两个正交邻居都必须是空的
                    if not self.is_free((cur[0] + dx, cur[1])) or \
                       not self.is_free((cur[0], cur[1] + dy)):
                        continue
                step = math.sqrt(2) if (dx and dy) else 1.0
                ng = gc + step
                if ng < g.get(nxt, 1e18):
                    g[nxt] = ng
                    came[nxt] = cur
                    heapq.heappush(open_heap, (ng + h(nxt), ng, nxt))
        return None

    # ------------------------------------------------------------ 路径
    def smooth(self, path: Sequence[Cell]) -> List[Cell]:
        """贪心视线平滑：能直连就跳过中间点，减少无谓的拐点。"""
        if len(path) <= 2:
            return list(path)
        out = [path[0]]
        i = 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1 and not self.line_of_sight(path[i], path[j]):
                j -= 1
            out.append(path[j])
            i = j
        return out

    def plan(self, start_xy: Point, goal_xy: Point,
             smooth: bool = True) -> Optional[List[Point]]:
        """世界坐标进、世界坐标出。无解返回 None。"""
        cells = self.astar(self.world_to_cell(*start_xy), self.world_to_cell(*goal_xy))
        if cells is None:
            return None
        if smooth:
            cells = self.smooth(cells)
        return [self.cell_to_world(c) for c in cells]

    def path_length(self, pts: Sequence[Point]) -> float:
        return sum(math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1))
