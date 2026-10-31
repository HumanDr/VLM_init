"""A* 全局路径规划节点：给一个目的地，输出一条绕开墙的路径（/plan）。

为什么需要它：
    控制器里的"goto"只是朝目标直着走 + 本地避障。目标在墙后面时，它只会顶着墙蹭。
    有了全局路径，控制器改成"跟着路径上的下一个路点走"，就能绕过去。

坐标：地图是**世界坐标**，而 `/odom` 从出生点算起，两者差一个平移
（默认出生在世界 (1.5, 1.5)）。节点里用 `map_offset_x/y` 做换算，
launch 会把 `x:=`/`y:=` 自动传进来，所以换出生点不用改别的地方。

发布：
    /plan   nav_msgs/Path          路径（odom 坐标系，含每个点的朝向）
    /map    nav_msgs/OccupancyGrid 静态地图（odom 系），给 rviz 看
"""
from __future__ import annotations

import math
import os
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
import numpy as np
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Empty

from .grid_map import GridMap


def _yaw_from_quat(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class PathPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__('path_planner_node')
        p = self.declare_parameter
        self.map_file = str(p('map_file', '').value)
        # file: 读离线地图（world 生成）；topic: 订阅 SLAM 实时地图
        self.map_source = str(p('map_source', 'file').value)
        self.goal_topic = p('goal_topic', '/goal_pose').value
        self.goal_cancel_topic = p('goal_cancel_topic', '/goal_cancel').value
        self.odom_topic = p('odom_topic', '/odom').value
        self.plan_topic = p('plan_topic', '/plan').value
        self.map_topic = p('map_topic', '/map').value
        self.inflation_radius = float(p('inflation_radius', 0.28).value)
        self.map_offset_x = float(p('map_offset_x', 1.5).value)
        self.map_offset_y = float(p('map_offset_y', 1.5).value)
        self.frame_id = str(p('frame_id', 'odom').value)
        self.map_wait_timeout = float(p('map_wait_timeout', 20.0).value)
        self.map_preview_topic = str(p('map_preview_topic', '/map_preview').value)
        self._fallback_map = False
        self._timeout_checked = False
        self.replan_period = float(p('replan_period', 2.0).value)
        self.odom_timeout = float(p('odom_timeout', 0.5).value)
        self.publish_map = bool(p('publish_map', True).value)
        self.smooth = bool(p('smooth', True).value)

        self._map: Optional[GridMap] = None
        self._planner_map: Optional[GridMap] = None
        self._map_rev = 0
        self._raw_offset = (self.map_offset_x, self.map_offset_y)
        if self.map_source == 'topic':
            # SLAM 的地图和位姿本来就在同一坐标系里，不需要再平移
            self.map_offset_x = self.map_offset_y = 0.0
            self.get_logger().info(
                f'地图来源: 订阅 {self.map_topic}（SLAM 实时建图，frame={self.frame_id}）')
            self.create_subscription(OccupancyGrid, self.map_topic,
                                     self._on_map_msg, 1)
            self._pub_preview = self.create_publisher(
                OccupancyGrid, self.map_preview_topic, 1)
            self.create_timer(self.map_wait_timeout, self._check_map_timeout)
        else:
            self._load_map()

        self._odom: Optional[Tuple[float, float]] = None
        self._odom_t = 0.0
        self._goal: Optional[Tuple[float, float]] = None      # 世界坐标
        self._dirty = True
        self._last_plan_t = 0.0
        self._last_fail_log = 0.0

        self._pub_plan = self.create_publisher(Path, self.plan_topic, 10)
        # SLAM 模式下地图由 SLAM 发布，这里不再重复发
        self._pub_map = (self.create_publisher(OccupancyGrid, self.map_topic, 1)
                         if (self.publish_map and self.map_source != 'topic') else None)
        self.create_subscription(PoseStamped, self.goal_topic, self._on_goal, 10)
        self.create_subscription(Empty, self.goal_cancel_topic, self._on_cancel, 10)
        self.create_subscription(Odometry, self.odom_topic, self._on_odom, 10)
        self.create_timer(0.2, self._tick)
        self.create_timer(1.0, self._publish_map_msg)
        self.get_logger().info(
            f'A* 规划节点启动: map={self.map_file or "(未配置)"} '
            f'inflation={self.inflation_radius}m offset=({self.map_offset_x},{self.map_offset_y}) '
            f'replan={self.replan_period}s')

    # ------------------------------------------------------------ 地图
    def _load_map(self) -> None:
        if not self.map_file or not os.path.isfile(self.map_file):
            self.get_logger().error(
                f'地图文件不存在: {self.map_file!r} → A* 不工作，'
                '控制器会退化为"直线奔向目标"。用 scripts/make_map_from_world.py 生成。')
            return
        self._map = GridMap.load(self.map_file)
        self._planner_map = self._map.inflated(self.inflation_radius)
        self.get_logger().info(
            f'地图 {self._map.width}x{self._map.height} @ {self._map.resolution}m, '
            f'origin={self._map.origin}, 障碍 {int(sum(sum(r) for r in self._map.grid))} 格')

    def _check_map_timeout(self) -> None:
        """SLAM 一直没出地图时，回退到离线地图，至少保证能规划/能看。"""
        if self._timeout_checked or self._map is not None:
            return
        self._timeout_checked = True
        if not self.map_file or not os.path.isfile(self.map_file):
            self.get_logger().warn(
                f'等了 {self.map_wait_timeout:.0f}s 没收到 {self.map_topic}，'
                '也没有离线地图可退回 → A* 暂时无法规划（控制器仍可直连目标）')
            return
        self._load_map()
        # 离线地图是世界坐标，SLAM 的 map 系原点在出生点 -> 要减掉出生点，
        # 否则 A* 会沿着"整体偏 1.5m"的地图规划，路径直接穿墙
        if self._map is not None and self._raw_offset != (0.0, 0.0):
            self._map = GridMap.from_array(
                self._map.grid, self._map.resolution,
                (self._map.origin[0] - self._raw_offset[0],
                 self._map.origin[1] - self._raw_offset[1]))
            self._planner_map = self._map.inflated(self.inflation_radius)
        self._fallback_map = True
        self._dirty = True
        self.get_logger().warn(
            f'等了 {self.map_wait_timeout:.0f}s 没收到 {self.map_topic}'
            f'（RTAB-Map 可能没拿到图像/雷达），已回退离线地图 '
            f'{os.path.basename(self.map_file)}，预览发到 {self.map_preview_topic}')

    def _on_map_msg(self, msg: OccupancyGrid) -> None:
        """把 SLAM 的 /map 转成 A* 用的栅格（0.05m 太细，降采样到 ~0.1m）。"""
        w, h = int(msg.info.width), int(msg.info.height)
        if w < 10 or h < 10:
            return
        step = 1 if msg.info.resolution >= 0.09 else 2
        arr = np.asarray(msg.data, dtype=np.int8).reshape(h, w)[::step, ::step]
        res = float(msg.info.resolution) * step
        occ = arr > 50                     # 未知(-1)/空闲(0) 都当可走
        if not occ.any():
            return
        self._map = GridMap.from_array(occ, res,
                                       (float(msg.info.origin.position.x),
                                        float(msg.info.origin.position.y)))
        self._planner_map = self._map.inflated(self.inflation_radius)
        self._dirty = True
        self._map_rev += 1
        if self._fallback_map:
            self._fallback_map = False
            self.get_logger().info(f'{self.map_topic} 已就绪，切回 SLAM 实时地图')
        if self._map_rev % 10 == 1:
            self.get_logger().info(
                f'SLAM 地图更新: {self._map.width}x{self._map.height} @ {res:.2f}m, '
                f'障碍 {int(occ.sum())} 格')

    # ------------------------------------------------------------ 回调
    def _on_odom(self, msg: Odometry) -> None:
        self._odom = (float(msg.pose.pose.position.x),
                      float(msg.pose.pose.position.y))
        self._odom_t = self.get_clock().now().nanoseconds / 1e9

    def _on_goal(self, msg: PoseStamped) -> None:
        self._goal = (float(msg.pose.position.x) + self.map_offset_x,
                      float(msg.pose.position.y) + self.map_offset_y)
        self._dirty = True
        self.get_logger().info(
            f'收到目的地 (odom {msg.pose.position.x:.2f},{msg.pose.position.y:.2f}) '
            f'-> 世界坐标 ({self._goal[0]:.2f},{self._goal[1]:.2f})')

    def _on_cancel(self, _msg: Empty) -> None:
        self._goal = None
        self._dirty = False
        self._pub_plan.publish(Path())          # 空路径 = 控制器回退到直线

    # ------------------------------------------------------------ 规划
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _tick(self) -> None:
        if self._planner_map is None or self._goal is None or self._odom is None:
            return
        now = self._now()
        if not self._dirty and now - self._last_plan_t < self.replan_period:
            return
        if now - self._odom_t > self.odom_timeout:
            return
        self._dirty = False
        self._last_plan_t = now

        start = (self._odom[0] + self.map_offset_x,
                 self._odom[1] + self.map_offset_y)
        pts = self._planner_map.plan(start, self._goal, smooth=self.smooth)
        if not pts:
            if now - self._last_fail_log > 5.0:
                self._last_fail_log = now
                self.get_logger().warn(
                    f'A* 无解: {tuple(round(v, 2) for v in start)} -> '
                    f'{tuple(round(v, 2) for v in self._goal)}（目标可能在障碍里）')
            self._publish_path([])
            return
        self._publish_path(pts)

    def _publish_path(self, pts: List[Tuple[float, float]]) -> None:
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        for i, (wx, wy) in enumerate(pts):
            ps = PoseStamped()
            ps.header = msg.header
            # 世界坐标 -> odom 坐标
            ps.pose.position.x = float(wx - self.map_offset_x)
            ps.pose.position.y = float(wy - self.map_offset_y)
            # 朝向指向下一个点，rviz 里能看出行进方向
            if i + 1 < len(pts):
                dx = pts[i + 1][0] - wx
                dy = pts[i + 1][1] - wy
            elif i > 0:
                dx = wx - pts[i - 1][0]
                dy = wy - pts[i - 1][1]
            else:
                dx, dy = 1.0, 0.0
            yaw = math.atan2(dy, dx)
            ps.pose.orientation.z = math.sin(yaw / 2.0)
            ps.pose.orientation.w = math.cos(yaw / 2.0)
            msg.poses.append(ps)
        self._pub_plan.publish(msg)

    def _publish_map_msg(self) -> None:
        pub = self._pub_map
        if self.map_source == 'topic':
            # SLAM 模式只在回退到离线地图时才预览，避免和 SLAM 抢 /map
            pub = self._pub_preview if self._fallback_map else None
        if pub is None or self._map is None:
            return
        m = self._map
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.info.resolution = m.resolution
        msg.info.width = m.width
        msg.info.height = m.height
        # 地图是世界坐标，换算到 odom
        msg.info.origin.position.x = m.origin[0] - self.map_offset_x
        msg.info.origin.position.y = m.origin[1] - self.map_offset_y
        msg.info.origin.orientation.w = 1.0
        msg.data = [100 if cell else 0 for row in m.grid for cell in row]
        pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
