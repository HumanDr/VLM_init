"""反应式避障规划器（纯 Python，无 ROS 依赖，可单测）。

它在"感知-决策-执行"里承担决策：把 VLM 的低频语义判断、深度的几何距离、
IMU 的姿态/航向，合成一条 20Hz 的 (linear, angular) 指令。

为什么需要这一层而不是"VLM 直接出速度"：
  * VLM 1~2Hz，且偶发幻觉/抖动；直接映射会一顿一顿地撞；
  * 用"承诺窗口(hold)"把一次避开动作保持一段时间，避免抖动；
  * 用 IMU 做航向保持(直线行驶不跑偏) 与 倾覆急停；
  * 用看门狗处理"模型挂了/相机断了"的降级。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional, Tuple

INF = float('inf')


def wrap_angle(a: float) -> float:
    """把角度包裹到 [-pi, pi]。"""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


@dataclass
class PlannerParams:
    # --- 速度 ---
    max_linear: float = 0.45
    cruise_linear: float = 0.35
    creep_linear: float = 0.12
    max_angular: float = 1.20
    avoid_angular: float = 0.85
    # --- 深度安全网（阈值已按"贴得更近才躲"调低） ---
    stop_distance: float = 0.15        # 硬停车
    slow_distance: float = 0.80        # 开始减速
    obstacle_distance: float = 0.35    # 触发转向
    release_margin: float = 0.25       # 滞回解除距离 = obstacle_distance + 本值
    turn_min_clearance: float = 0.35   # 两侧都小于它才算"无路可走"
    side_margin: float = 0.12
    side_push_gain: float = 0.60
    side_push_max: float = 0.25
    side_push_distance: float = 0.75
    clear_path_distance: float = 0.55  # 前方净空大于它：侧向障碍只转向不减速
    # 侧向避让的"预算"：转过一定角度/时间就必须结束，否则会以恒定角速度绕圈
    lateral_swerve_max_angle: float = 0.4363  # rad（25°），累计转过多少就停
    lateral_swerve_max_time: float = 1.0      # s
    lateral_cooldown: float = 2.5            # s，两次侧向避让之间的间隔
    # --- 决策时序 ---
    hold_time: float = 1.2
    vlm_risk_tau: float = 2.5
    unknown_risk: float = 0.35
    obstacle_risk_threshold: float = 0.70
    prefer_side: int = 1
    # --- 降级 ---
    vlm_stale_timeout: float = 3.0
    stale_creep: float = 0.12
    stop_after_stale: float = 6.0
    sensor_timeout: float = 0.5
    # --- IMU ---
    tilt_estop: float = 0.90
    heading_hold: bool = True
    heading_kp: float = 0.9
    heading_max_angular: float = 0.35
    heading_deadband: float = 0.04
    heading_relock: float = 0.60
    # --- 平滑与自救 ---
    smoothing_alpha: float = 0.45
    recovery_after: float = 4.0
    recovery_time: float = 2.0
    recovery_angular: float = 0.80
    recovery_linear: float = -0.06

@dataclass
class PlannerInputs:
    now: float = 0.0
    dt: float = 0.05
    # VLM 语义通道
    vlm_ok: bool = False
    vlm_obstacle: bool = False
    vlm_risk: float = 0.0
    vlm_direction: str = 'none'
    vlm_action: str = 'go_straight'
    vlm_age: float = INF
    vlm_latency_s: float = 0.0
    # 深度几何通道
    sectors_valid: bool = False
    sectors_age: float = INF
    left: float = 5.0
    center: float = 5.0
    right: float = 5.0
    # IMU
    imu_valid: bool = False
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0


@dataclass
class PlannerOutputs:
    linear: float = 0.0
    angular: float = 0.0
    state: str = 'INIT'
    risk: float = 0.0
    clearance_center: float = 5.0
    watchdog_scale: float = 1.0
    degraded: bool = False
    note: str = ''


class ReactivePlanner:
    """状态机：CRUISE / CAUTION / AVOID_LEFT / AVOID_RIGHT / STOP / RECOVERY / ESTOP。"""

    def __init__(self, params: Optional[PlannerParams] = None) -> None:
        self.p = params or PlannerParams()
        self.reset()

    def reset(self) -> None:
        self._prev_linear = 0.0
        self._prev_angular = 0.0
        self._last_state = 'INIT'
        self._target_yaw: Optional[float] = None
        self._commit_side = 0
        self._commit_until = 0.0
        self._stop_since: Optional[float] = None
        self._recovery_until = 0.0
        self._first_sense_time: Optional[float] = None
        self._cmded = False
        self._near = False
        self._lateral_start: Optional[float] = None
        self._lateral_yaw0 = 0.0
        self._lateral_block_until = 0.0

    # ------------------------------------------------------------ helpers
    def _vlm_risk(self, inp: PlannerInputs) -> float:
        if not inp.vlm_ok or not inp.vlm_obstacle:
            return 0.0
        age = max(0.0, inp.vlm_age)
        # 衰减常数不能短于"模型两帧之间的间隔"：慢推理（CPU/大模型）时
        # 若仍按 2.5s 衰减，模型刚报的障碍在控制器读到时已经衰减成 0。
        tau = max(self.p.vlm_risk_tau, 2.0 * max(0.0, inp.vlm_latency_s))
        decay = math.exp(-age / max(1e-3, tau))
        return clamp(inp.vlm_risk * decay, 0.0, 1.0)

    def _geo(self, inp: PlannerInputs) -> Tuple[bool, float]:
        """返回 (深度是否可用, 几何风险)。"""
        usable = inp.sectors_valid and inp.sectors_age <= self.p.sensor_timeout
        if not usable:
            return False, 0.0
        span = max(1e-3, self.p.slow_distance - self.p.stop_distance)
        return True, clamp((self.p.slow_distance - inp.center) / span, 0.0, 1.0)

    def _choose_side(self, inp: PlannerInputs, has_depth: bool, risk: float,
                     now: float = 0.0) -> int:
        """返回 +1(向左打角) / -1(向右打角) / 0(没有安全侧 -> 停)。"""
        p = self.p
        # 承诺窗口内沿用上次选择（只要那一侧仍然够空），否则会每帧左右横跳，
        # 平滑后角速度抵消成 ~0，车只会原地磨。
        if self._commit_side != 0 and now < self._commit_until:
            clearance = inp.left if self._commit_side > 0 else inp.right
            if (not has_depth) or clearance >= p.turn_min_clearance:
                return self._commit_side
        if has_depth:
            left, right = inp.left, inp.right
            # 两侧都太窄：原地停，交给 recovery 兜底
            if left < p.turn_min_clearance and right < p.turn_min_clearance:
                return 0
            if left > right + p.side_margin:
                return 1
            if right > left + p.side_margin:
                return -1
            # 两侧都够宽：优先听语义，其次用默认偏好侧
            if inp.vlm_ok and (inp.vlm_direction or 'none').lower() in ('left', 'right'):
                return -1 if (inp.vlm_direction or '').lower() == 'left' else 1
            if inp.vlm_ok and (inp.vlm_action or '').lower() in ('turn_left', 'turn_right'):
                return 1 if (inp.vlm_action or '').lower() == 'turn_left' else -1
            return int(math.copysign(1, p.prefer_side)) if p.prefer_side else 1
        direction = (inp.vlm_direction or 'none').lower()
        action = (inp.vlm_action or 'go_straight').lower()
        if inp.vlm_ok and direction in ('left', 'right'):
            # 障碍在左 -> 向右（负角速度）
            return -1 if direction == 'left' else 1
        if inp.vlm_ok and action in ('turn_left', 'turn_right'):
            return 1 if action == 'turn_left' else -1
        if inp.vlm_ok and action == 'stop':
            return 0
        return int(math.copysign(1, p.prefer_side)) if p.prefer_side else 1

    # ------------------------------------------------------------ main
    def update(self, inp: PlannerInputs) -> PlannerOutputs:
        p = self.p
        now = inp.now
        if self._first_sense_time is None:
            self._first_sense_time = now

        has_depth, geo_risk = self._geo(inp)
        risk_vlm = self._vlm_risk(inp)
        vlm_fresh = inp.vlm_ok and inp.vlm_age <= p.vlm_stale_timeout
        degraded = not vlm_fresh

        # ---------- 0. 倾覆急停（最高优先级） ----------
        if inp.imu_valid and max(abs(inp.roll), abs(inp.pitch)) > p.tilt_estop:
            self._commit_side = 0
            self._stop_since = None
            return self._emit(inp, 0.0, 0.0, 'ESTOP', 1.0, inp.center, 0.0,
                              'IMU 姿态超限', degraded=degraded)

        # ---------- 1. 风险融合 ----------
        # 正前方被挡住（模型说 center）-> 一定当障碍，且风险不低于 0.7
        center_block = bool(inp.vlm_ok and inp.vlm_obstacle
                            and inp.vlm_direction == 'center')

        # 侧向语义只有在"可执行"时才生效，否则会把车带着绕圈：
        #   a) 必须前方通畅（否则该减速）
        #   b) 必须被深度佐证：模型报的那一侧确实更窄
        #   c) 必须有预算：一次侧向避让最多转 max_angle / max_time，之后进入冷却
        lateral_only = False
        lateral_claim = bool(inp.vlm_ok and inp.vlm_obstacle
                             and inp.vlm_direction in ('left', 'right')
                             and has_depth and inp.center > p.clear_path_distance)
        if lateral_claim:
            if inp.vlm_direction == 'left':
                corroborated = inp.left <= inp.right + p.side_margin
            else:
                corroborated = inp.right <= inp.left + p.side_margin
            if not corroborated or now < self._lateral_block_until:
                lateral_only = False
            else:
                if self._lateral_start is None:
                    self._lateral_start = now
                    self._lateral_yaw0 = inp.yaw
                turned = (abs(wrap_angle(inp.yaw - self._lateral_yaw0))
                          if inp.imu_valid else 0.0)
                if (now - self._lateral_start > p.lateral_swerve_max_time
                        or turned > p.lateral_swerve_max_angle):
                    lateral_only = False
                    self._lateral_start = None
                    self._lateral_block_until = now + p.lateral_cooldown
                else:
                    lateral_only = True
        else:
            self._lateral_start = None

        # 只有"可执行"的语义才参与风险；否则忽略这条侧向报告
        vlm_actionable = bool(inp.vlm_ok and inp.vlm_obstacle
                              and (inp.vlm_direction == 'center' or lateral_only))
        risk_vlm_eff = risk_vlm if vlm_actionable else 0.0
        if lateral_only:
            risk = geo_risk if has_depth else 0.0
        elif has_depth:
            risk = max(risk_vlm_eff, geo_risk)
        else:
            risk = risk_vlm_eff
        if center_block:
            risk = max(risk, 0.7)
        no_sense = (not vlm_fresh) and (not has_depth)
        if no_sense:
            risk = max(risk, p.unknown_risk)
            # 从没收到过任何感知、或感知长时间缺失：宁可停，也不能"盲开"
            if inp.vlm_age == INF:
                return self._emit(inp, 0.0, 0.0, 'STOP', risk, inp.center, 0.0,
                                  '无任何感知输入', degraded=True)
            if inp.vlm_age > p.stop_after_stale:
                return self._emit(inp, 0.0, 0.0, 'STOP', risk, inp.center, 0.0,
                                  '感知长时间不可用', degraded=True)

        # ---------- 2. 速度上限 ----------
        cap = p.cruise_linear
        if has_depth:
            cap = min(cap, max(0.0, (inp.center - p.stop_distance) * 0.9))
        if risk > 0.0:
            cap = min(cap, p.cruise_linear * (1.0 - 0.85 * risk))
            cap = min(cap, p.max_linear)
        if degraded and not has_depth:
            cap = min(cap, p.stale_creep)     # 感知掉线后短暂蠕行，随后上面会直接停
        elif degraded:
            # VLM 不可用但有深度：靠几何通道继续走，但要更慢
            cap = min(cap, p.cruise_linear * 0.7)
        cap = max(0.0, min(cap, p.max_linear))

        # ---------- 3. 是否认为有障碍 ----------
        obstacle = risk >= p.obstacle_risk_threshold or center_block
        if has_depth:
            # 滞回：1.0m 触发、1.3m 才解除，避免在阈值附近 CRUISE/AVOID 反复横跳
            if inp.center < p.obstacle_distance:
                self._near = True
            elif inp.center > p.obstacle_distance + p.release_margin:
                self._near = False
            obstacle = obstacle or self._near or lateral_only

        # ---------- 4. 转向决策 + 承诺窗口 ----------
        side = 0
        if obstacle:
            side = self._choose_side(inp, has_depth, risk, now)
            if side != 0:
                self._commit_side = side
                self._commit_until = now + p.hold_time
        elif now < self._commit_until and self._commit_side != 0:
            side = self._commit_side       # 视野刚变干净时继续保持转向
        else:
            self._commit_side = 0

        # ---------- 5. 自救（卡死后原地转向脱困） ----------
        if now < self._recovery_until:
            side_r = 1 if inp.left >= inp.right else -1
            return self._emit(inp, p.recovery_linear, side_r * p.recovery_angular,
                              'RECOVERY', risk, inp.center, 1.0, '原地脱困',
                              degraded=degraded)

        linear = 0.0
        angular = 0.0
        state = 'CRUISE'

        if side != 0 and (obstacle or now < self._commit_until):
            # 避让动作要么由当前障碍触发，要么在承诺窗口内保持
            angular = side * p.avoid_angular * (0.55 + 0.45 * min(1.0, risk / 0.8))
            linear = cap if lateral_only else min(cap, p.creep_linear)
            state = 'AVOID_LEFT' if side > 0 else 'AVOID_RIGHT'
            self._stop_since = None
        elif obstacle:
            linear, angular, state = 0.0, 0.0, 'STOP'
            if self._stop_since is None:
                self._stop_since = now
            elif now - self._stop_since > p.recovery_after:
                self._recovery_until = now + p.recovery_time
                self._stop_since = None
                self._commit_side = 0
                side_r = int(math.copysign(1, p.prefer_side)) if p.prefer_side else 1
                return self._emit(inp, p.recovery_linear,
                                  side_r * p.recovery_angular, 'RECOVERY',
                                  risk, inp.center, 1.0, '卡死自救',
                                  degraded=degraded)
        else:
            linear = cap
            state = 'CRUISE' if risk < 0.15 else 'CAUTION'
            self._stop_since = None

        # ---------- 6. IMU 航向保持（只在直行时生效） ----------
        if state in ('CRUISE', 'CAUTION') and p.heading_hold and inp.imu_valid:
            if self._last_state in ('AVOID_LEFT', 'AVOID_RIGHT'):
                self._target_yaw = inp.yaw        # 避障结束后重新锁定，别把车头掰回去
            if self._target_yaw is None:
                self._target_yaw = inp.yaw
            err = wrap_angle(inp.yaw - self._target_yaw)
            if abs(err) > p.heading_relock:
                self._target_yaw = inp.yaw
                err = 0.0
            if abs(err) > p.heading_deadband:
                angular += clamp(-p.heading_kp * err,
                                 -p.heading_max_angular, p.heading_max_angular)

        # ---------- 6b. 侧向净空推离：贴墙时把车头推回通道中线 ----------
        if (state in ('CRUISE', 'CAUTION') and has_depth
                and p.side_push_gain > 0.0 and min(inp.left, inp.right) < p.side_push_distance):
            push = p.side_push_gain * (inp.left - inp.right) / max(1e-3, inp.left + inp.right)
            push = clamp(push, -p.side_push_max, p.side_push_max)
            angular += push

        # ---------- 7. 平滑与限幅 ----------
        if not self._cmded:
            smooth_lin, smooth_ang = linear, angular
            self._cmded = True
        elif state in ('STOP', 'RECOVERY', 'ESTOP'):
            smooth_lin, smooth_ang = linear, angular
        else:
            a = clamp(p.smoothing_alpha, 0.0, 1.0)
            smooth_lin = a * linear + (1.0 - a) * self._prev_linear
            smooth_ang = a * angular + (1.0 - a) * self._prev_angular
        if self._last_state in ('AVOID_LEFT', 'AVOID_RIGHT') and state in ('CRUISE', 'CAUTION'):
            smooth_ang = angular            # 避让结束立刻把方向交还航向保持，避免过冲
        smooth_lin = clamp(smooth_lin, -p.max_linear, p.max_linear)
        smooth_ang = clamp(smooth_ang, -p.max_angular, p.max_angular)

        wd = 1.0
        if no_sense:
            wd = 0.0
        elif degraded and not has_depth:
            wd = p.stale_creep / max(1e-3, p.cruise_linear)
        note = '' if not degraded else ('仅深度通道' if has_depth else '感知不可用')

        return self._emit(inp, smooth_lin, smooth_ang, state, risk, inp.center, wd,
                          note, degraded=degraded)

    # ------------------------------------------------------------ emit
    def _emit(self, inp: PlannerInputs, lin: float, ang: float, state: str,
              risk: float, clearance: float, wd: float,
              note: str = '', degraded: bool = False) -> PlannerOutputs:
        p = self.p
        lin = clamp(lin, -p.max_linear, p.max_linear)
        ang = clamp(ang, -p.max_angular, p.max_angular)
        out = PlannerOutputs(
            linear=float(lin), angular=float(ang), state=state,
            risk=float(clamp(risk, 0.0, 1.0)),
            clearance_center=float(clamp(clearance, 0.0, 99.0)),
            watchdog_scale=float(clamp(wd, 0.0, 1.0)),
            degraded=bool(degraded),
            note=note)
        self._prev_linear, self._prev_angular = out.linear, out.angular
        self._last_state = state
        return out
