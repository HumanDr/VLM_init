"""避障控制节点：融合 VLM / 深度扇区 / IMU -> /cmd_vel。

这是"决策-执行"的唯一出口，所有安全约束都在这里收口：
  * 20Hz 定时器产生速度指令（VLM 慢不会拖慢控制回路）；
  * 任何输入缺失都有明确降级策略（见 docs/architecture.md）；
  * 退出时主动发一次零速度，真机上防止"节点死了车还在跑"。
"""
from __future__ import annotations

import time
from dataclasses import fields

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from vlm_avoid_interfaces.msg import DriveStatus, ImuState, SectorScan, VlmObstacle

from .planner import (INF, PlannerInputs, PlannerParams, ReactivePlanner,
                      clamp)


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
        self._last_tick = time.monotonic()
        self._last_state = ''
        self._log_t = 0.0

        self.create_subscription(VlmObstacle, self.vlm_topic, self._on_vlm, 10)
        self.create_subscription(SectorScan, self.sector_topic, self._on_sectors,
                                 qos_profile_sensor_data)
        self.create_subscription(ImuState, self.imu_topic, self._on_imu, 10)

        self._pub_cmd = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self._pub_status = self.create_publisher(DriveStatus, self.status_topic, 10)

        self.add_on_set_parameters_callback(self._on_set_params)

        self._timer = self.create_timer(1.0 / max(1.0, self.rate_hz), self._tick)
        self.get_logger().info(
            f'避障控制器启动: cmd={self.cmd_vel_topic} vlm={self.vlm_topic} '
            f'sectors={self.sector_topic} imu={self.imu_topic} '
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
        sectors_valid = (self._sectors is not None and self._sectors.valid
                         and sec_age <= p.sensor_timeout)
        imu_age = now - self._imu_t if self._imu is not None else INF
        imu_valid = (self._imu is not None and self._imu.valid
                     and imu_age <= p.sensor_timeout)

        inp = PlannerInputs(
            now=now, dt=dt,
            vlm_ok=vlm_ok,
            vlm_obstacle=bool(self._vlm.obstacle) if self._vlm else False,
            vlm_risk=float(self._vlm.risk) if self._vlm else 0.0,
            vlm_direction=self._vlm.direction if self._vlm else 'none',
            vlm_action=self._vlm.action if self._vlm else 'go_straight',
            vlm_age=vlm_age,
            vlm_latency_s=vlm_latency_s,
            sectors_valid=sectors_valid,
            sectors_age=sec_age,
            left=float(self._sectors.left) if self._sectors else 5.0,
            center=float(self._sectors.center) if self._sectors else 5.0,
            right=float(self._sectors.right) if self._sectors else 5.0,
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
