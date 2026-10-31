"""避障控制节点：融合 VLM / 深度扇区 / IMU -> /cmd_vel。

这是"决策-执行"的唯一出口，所有安全约束都在这里收口：
  * 20Hz 定时器产生速度指令（VLM 慢不会拖慢控制回路）；
  * 任何输入缺失都有明确降级策略（见 docs/architecture.md）；
  * 退出时主动发一次零速度，真机上防止"节点死了车还在跑"。
"""
from __future__ import annotations

import math
import time
from dataclasses import fields
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Empty
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from vlm_avoid_interfaces.msg import DriveStatus, ImuState, SectorScan, VlmObstacle

from .planner import (INF, PlannerInputs, PlannerParams, ReactivePlanner,
                      clamp, wrap_angle)


def _yaw_from_quat(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class AvoidanceController(Node):
    def __init__(self) -> None:
        super().__init__('avoidance_controller')
        p = self.declare_parameter

        params = PlannerParams()
        for f in fields(params):
            val = p(f.name, getattr(params, f.name)).value
            setattr(params, f.name, type(getattr(params, f.name))(val))
        self.planner = ReactivePlanner(params)
        self.params = params

        self.cmd_vel_topic = p('cmd_vel_topic', '/cmd_vel').value
        self.vlm_topic = p('vlm_topic', '/perception/vlm/result').value
        self.sector_topic = p('sector_topic', '/perception/sectors/result').value
        self.imu_topic = p('imu_state_topic', '/state/imu').value
        self.scan_sector_topic = p('scan_sector_topic',
                                   '/perception/scan_sectors').value
        self.odom_topic = p('odom_topic', '/odom').value
        self.goal_topic = p('goal_topic', '/goal_pose').value
        self.goal_cancel_topic = p('goal_cancel_topic', '/goal_cancel').value
        self.plan_topic = p('plan_topic', '/plan').value
        self.waypoint_tolerance = float(p('waypoint_tolerance', 0.35).value)
        self.odom_timeout = float(p('odom_timeout', 0.5).value)
        self.fallback_odom_topic = p('fallback_odom_topic', '/wheel_odom').value
        self.fallback_speed_scale = float(p('fallback_speed_scale', 0.6).value)
        self.status_topic = p('status_topic', '/vlm_avoid/status').value
        self.rate_hz = float(p('publish_rate_hz', 20.0).value)
        self.enable = bool(p('enable', True).value)
        self.require_imu = bool(p('require_imu', False).value)

        self._vlm: VlmObstacle | None = None
        self._vlm_t = 0.0
        self._sectors: SectorScan | None = None
        self._sectors_t = 0.0
        self._imu: ImuState | None = None
        self._imu_t = 0.0
        self._scan: SectorScan | None = None
        self._scan_t = 0.0
        self._fb_xy: Optional[Tuple[float, float]] = None
        self._fb_yaw = 0.0
        self._fb_t = 0.0
        self._fallback_active = False
        self._last_move_pos: Optional[Tuple[float, float]] = None
        self._last_move_t = 0.0
        self._stuck = False
        self._odom_xy: Optional[Tuple[float, float]] = None
        self._odom_yaw = 0.0
        self._odom_t = 0.0
        self._goal: Optional[Tuple[float, float]] = None
        self._goal_t = 0.0
        self._path: list = []          # A* 规划的路径（odom 系）
        self._path_t = 0.0
        self._last_tick = time.monotonic()
        self._last_state = ''
        self._log_t = 0.0

        self.create_subscription(VlmObstacle, self.vlm_topic, self._on_vlm, 10)
        self.create_subscription(SectorScan, self.sector_topic, self._on_sectors,
                                 qos_profile_sensor_data)
        self.create_subscription(ImuState, self.imu_topic, self._on_imu, 10)
        self.create_subscription(SectorScan, self.scan_sector_topic, self._on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, self.odom_topic, self._on_odom, 10)
        if self.fallback_odom_topic and self.fallback_odom_topic != self.odom_topic:
            self.create_subscription(Odometry, self.fallback_odom_topic,
                                     self._on_fallback_odom, 10)
        self.create_subscription(PoseStamped, self.goal_topic, self._on_goal, 10)
        self.create_subscription(Empty, self.goal_cancel_topic, self._on_goal_cancel, 10)
        self.create_subscription(Path, self.plan_topic, self._on_plan, 10)

        self._pub_cmd = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self._pub_status = self.create_publisher(DriveStatus, self.status_topic, 10)

        self.add_on_set_parameters_callback(self._on_set_params)

        self._timer = self.create_timer(1.0 / max(1.0, self.rate_hz), self._tick)
        self.get_logger().info(
            f'避障控制器启动: cmd={self.cmd_vel_topic} vlm={self.vlm_topic} '
            f'depth={self.sector_topic} scan={self.scan_sector_topic} '
            f'imu={self.imu_topic} odom={self.odom_topic} goal={self.goal_topic} '
            f'plan={self.plan_topic} '
            f'{self.rate_hz:.0f}Hz enable={self.enable}')

    # ------------------------------------------------------------ params
    def _on_set_params(self, params):
        """支持 ros2 param set 动态调参：改完立刻生效，不用重启。"""
        for item in params:
            if item.name in ('enable', 'require_imu'):
                setattr(self, item.name, bool(item.value))
                self.get_logger().info(f'参数 {item.name} -> {bool(item.value)}')
                continue
            if hasattr(self.params, item.name):
                cur = getattr(self.params, item.name)
                try:
                    val = type(cur)(item.value)
                except (TypeError, ValueError):
                    return SetParametersResult(
                        successful=False, reason=f'{item.name} 类型不符')
                setattr(self.params, item.name, val)
                self.get_logger().info(f'参数 {item.name} -> {val}')
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------ callbacks
    def _now(self) -> float:
        return time.monotonic()

    def _on_vlm(self, msg: VlmObstacle) -> None:
        self._vlm, self._vlm_t = msg, self._now()

    def _on_sectors(self, msg: SectorScan) -> None:
        self._sectors, self._sectors_t = msg, self._now()

    def _on_imu(self, msg: ImuState) -> None:
        self._imu, self._imu_t = msg, self._now()

    def _on_scan(self, msg: SectorScan) -> None:
        self._scan, self._scan_t = msg, self._now()

    def _on_odom(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        q = pose.orientation
        # 四元数 -> 偏航角（只取平面分量）
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._odom_xy = (float(pose.position.x), float(pose.position.y))
        self._odom_yaw = math.atan2(siny, cosy)
        self._odom_t = self._now()

    def _on_fallback_odom(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        self._fb_xy = (float(pose.position.x), float(pose.position.y))
        self._fb_yaw = _yaw_from_quat(pose.orientation)
        self._fb_t = self._now()

    def _on_goal(self, msg: PoseStamped) -> None:
        self._goal = (float(msg.pose.position.x), float(msg.pose.position.y))
        self._goal_t = self._now()
        self._path, self._path_t = [], 0.0     # 旧路径作废，等新的 A* 结果
        self.get_logger().info(
            f'收到目的地: x={self._goal[0]:.2f} y={self._goal[1]:.2f} '
            f'（frame={msg.header.frame_id or "odom"}），开始导航')

    def _on_goal_cancel(self, _msg: Empty) -> None:
        if self._goal is not None:
            self.get_logger().info('目的地已取消，恢复自主巡航')
        self._goal = None
        self._path, self._path_t = [], 0.0

    def _on_plan(self, msg: Path) -> None:
        self._path = [(float(p.pose.position.x), float(p.pose.position.y))
                      for p in msg.poses]
        self._path_t = self._now()

    # ------------------------------------------------------------ tick
    def _tick(self) -> None:
        now = self._now()
        dt = clamp(now - self._last_tick, 0.005, 0.5)
        self._last_tick = now
        p = self.params

        vlm_age = now - self._vlm_t if self._vlm is not None else INF
        vlm_latency_s = (float(self._vlm.latency_ms) / 1000.0) if self._vlm else 0.0
        # 慢模型（CPU 推理十几秒）两帧间隔本就超过 stale_timeout，按延迟放宽，
        # 否则永远是 stale；真挂掉时节点会发 stale=True，仍然会被判为不可用。
        stale_limit = max(p.vlm_stale_timeout, 3.0 * vlm_latency_s)
        vlm_ok = (self._vlm is not None and not self._vlm.stale
                  and vlm_age <= stale_limit)
        sec_age = now - self._sectors_t if self._sectors is not None else INF
        depth_valid = (self._sectors is not None and self._sectors.valid
                       and sec_age <= p.sensor_timeout)
        scan_age = now - self._scan_t if self._scan is not None else INF
        scan_valid = (self._scan is not None and self._scan.valid
                      and scan_age <= p.sensor_timeout)

        # 深度与雷达逐项取最小值：哪个更保守听哪个
        if depth_valid and scan_valid:
            left = min(self._sectors.left, self._scan.left)
            center = min(self._sectors.center, self._scan.center)
            right = min(self._sectors.right, self._scan.right)
        elif scan_valid:
            left, center, right = self._scan.left, self._scan.center, self._scan.right
        elif depth_valid:
            left, center, right = (self._sectors.left, self._sectors.center,
                                   self._sectors.right)
        else:
            left = center = right = 5.0
        sectors_valid = depth_valid or scan_valid
        sectors_age = min(sec_age, scan_age) if sectors_valid else INF
        imu_age = now - self._imu_t if self._imu is not None else INF
        imu_valid = (self._imu is not None and self._imu.valid
                     and imu_age <= p.sensor_timeout)

        # ---- 目的地：从 /odom 算距离与方位误差 ----
        odom_age = now - self._odom_t if self._odom_xy is not None else INF
        odom_valid = self._odom_xy is not None and odom_age <= self.odom_timeout
        fb_age = now - self._fb_t if self._fb_xy is not None else INF
        fb_valid = self._fb_xy is not None and fb_age <= self.odom_timeout
        pose_xy: Optional[Tuple[float, float]] = None
        pose_yaw = 0.0
        if odom_valid:
            pose_xy, pose_yaw = self._odom_xy, self._odom_yaw
            if self._fallback_active:
                self.get_logger().info('SLAM 位姿恢复，结束轮式里程计兜底')
            self._fallback_active = False
        elif fb_valid:
            if not self._fallback_active:
                self.get_logger().warn(
                    f'{self.odom_topic} 不可用，临时用 {self.fallback_odom_topic} '
                    f'兜底（限速 {self.fallback_speed_scale:.0%}），SLAM 恢复后自动切回')
            self._fallback_active = True
            pose_xy, pose_yaw = self._fb_xy, self._fb_yaw
        else:
            self._fallback_active = False
        goal_dist = 0.0
        goal_bearing = 0.0
        mode = 'cruise'
        if self._goal is not None:
            mode = 'goto'
            if pose_xy is not None:
                ox, oy = pose_xy
                goal_dist = math.hypot(self._goal[0] - ox, self._goal[1] - oy)
                # 有 A* 路径就盯"下一个路点"，没有就直连目标
                target = self._goal
                if self._path and (now - self._path_t) <= 10.0:
                    for (px, py) in self._path:
                        if math.hypot(px - ox, py - oy) > self.waypoint_tolerance:
                            target = (px, py)
                            break
                goal_bearing = wrap_angle(
                    math.atan2(target[1] - oy, target[0] - ox) - pose_yaw)

        # 卡住判定：3 秒内位移不到 8cm
        if pose_xy is not None:
            if (self._last_move_pos is None
                    or math.dist(pose_xy, self._last_move_pos) > 0.08):
                self._last_move_pos = pose_xy
                self._last_move_t = now
            self._stuck = (mode == 'goto' and now - self._last_move_t > 3.0)
        else:
            self._stuck = False

        inp = PlannerInputs(
            now=now, dt=dt,
            mode=mode,
            stuck=self._stuck,
            goal_dist=goal_dist,
            goal_bearing=goal_bearing,
            vlm_ok=vlm_ok,
            vlm_obstacle=bool(self._vlm.obstacle) if self._vlm else False,
            vlm_risk=float(self._vlm.risk) if self._vlm else 0.0,
            vlm_direction=self._vlm.direction if self._vlm else 'none',
            vlm_action=self._vlm.action if self._vlm else 'go_straight',
            vlm_age=vlm_age,
            vlm_latency_s=vlm_latency_s,
            sectors_valid=sectors_valid,
            sectors_age=sectors_age,
            left=float(left),
            center=float(center),
            right=float(right),
            imu_valid=imu_valid,
            roll=float(self._imu.roll) if self._imu else 0.0,
            pitch=float(self._imu.pitch) if self._imu else 0.0,
            yaw=float(self._imu.yaw) if self._imu else 0.0,
        )

        if not self.enable:
            # 关闭时也发布状态（state=DISABLED），方便 doctor/probe 诊断
            out = self.planner.update(inp)
            out.linear = out.angular = 0.0
            out.state = 'DISABLED'
            out.note = 'enable=false，被手动关闭'
            self._publish(0.0, 0.0)
            self._publish_status(out, inp)
            self._maybe_log(out, inp)
            return

        # 设了目的地、主位姿和兜底位姿都没有：才停
        if mode == 'goto' and pose_xy is None:
            out = self.planner.update(PlannerInputs(
                now=now, dt=dt, vlm_ok=False, sectors_valid=False, imu_valid=False))
            out.state = 'STOP'
            out.linear = out.angular = 0.0
            out.note = (f'已设目的地但 {self.odom_topic} 和 '
                        f'{self.fallback_odom_topic} 都不可用')
            self._publish(0.0, 0.0)
            self._publish_status(out, inp)
            self._maybe_log(out, inp)
            return

        # 真机安全开关：没有 IMU 就绝不动（仿真/纯视觉调试可关）
        if self.require_imu and not imu_valid:
            out = self.planner.update(PlannerInputs(
                now=now, dt=dt, vlm_ok=False, sectors_valid=False,
                imu_valid=False))
            out.state = 'ESTOP'
            out.linear = out.angular = 0.0
            out.note = 'require_imu 但 IMU 不可用'
            self._publish(0.0, 0.0)
            self._publish_status(out, inp)
            self._maybe_log(out, inp)
            return

        out = self.planner.update(inp)
        if self._fallback_active and out.linear > 0.0:
            out.linear *= self.fallback_speed_scale
            out.note = (out.note + ' ' if out.note else '') + '轮式兜底限速'
        if out.state == 'RECOVERY' and self._last_state != 'RECOVERY':
            self.get_logger().warn('检测到卡住，倒退转向自救中')
        self._publish(out.linear, out.angular)
        self._publish_status(out, inp)
        self._maybe_log(out, inp)

    def _publish(self, linear: float, angular: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self._pub_cmd.publish(msg)

    def _publish_status(self, out, inp: PlannerInputs) -> None:
        s = DriveStatus()
        s.header.stamp = self.get_clock().now().to_msg()
        s.header.frame_id = 'base_footprint'
        s.linear_cmd = out.linear
        s.angular_cmd = out.angular
        s.state = out.state
        s.risk = out.risk
        s.clearance_center = out.clearance_center
        s.yaw = inp.yaw
        s.vlm_stale = not inp.vlm_ok
        s.watchdog_scale = out.watchdog_scale
        s.distance_to_goal = float(inp.goal_dist) if inp.mode == 'goto' else -1.0
        self._pub_status.publish(s)

    def _maybe_log(self, out, inp: PlannerInputs) -> None:
        now = self._now()
        if out.state != self._last_state:
            self.get_logger().info(
                f'状态 {self._last_state or "INIT"} -> {out.state} '
                f'风险={out.risk:.2f} 前向净空={out.clearance_center:.2f}m '
                f'cmd=(v={out.linear:.2f}, w={out.angular:.2f}) {out.note}')
            self._last_state = out.state
            self._log_t = now
        elif out.degraded and now - self._log_t > 5.0:
            self.get_logger().warn(
                f'感知降级中（{out.note}）vlm_age={inp.vlm_age if inp.vlm_age != INF else -1:.1f}s '
                f'cmd=(v={out.linear:.2f}, w={out.angular:.2f})')
            self._log_t = now

    def destroy_node(self) -> bool:
        try:
            self._publish(0.0, 0.0)
        except Exception:  # noqa: BLE001
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AvoidanceController()
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
