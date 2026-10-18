"""规划器行为单测：覆盖核心状态机与安全降级。"""
import math

import pytest

from vlm_control.planner import (INF, PlannerInputs, PlannerParams,
                                 ReactivePlanner, wrap_angle)


def make_planner(**kw):
    return ReactivePlanner(PlannerParams(**kw))


def free_inputs(t=0.0, **kw):
    base = dict(now=t, dt=0.05, vlm_ok=True, vlm_obstacle=False, vlm_risk=0.03,
                vlm_direction='none', vlm_action='go_straight', vlm_age=0.1,
                sectors_valid=True, sectors_age=0.05,
                left=3.0, center=3.0, right=3.0,
                imu_valid=True, roll=0.0, pitch=0.0, yaw=0.0)
    base.update(kw)
    return PlannerInputs(**base)


def test_wrap_angle():
    assert wrap_angle(0.0) == pytest.approx(0.0)
    assert abs(wrap_angle(3 * math.pi)) == pytest.approx(math.pi, abs=1e-9)
    assert abs(wrap_angle(-3 * math.pi)) == pytest.approx(math.pi, abs=1e-9)
    assert wrap_angle(2 * math.pi + 0.1) == pytest.approx(0.1)


def test_cruise_on_clear_path():
    pl = make_planner()
    out = pl.update(free_inputs())
    assert out.state == 'CRUISE'
    assert out.linear > 0.2
    assert abs(out.angular) < 0.05
    assert out.watchdog_scale == pytest.approx(1.0)


def test_avoid_center_obstacle():
    pl = make_planner()
    out = pl.update(free_inputs(vlm_obstacle=True, vlm_risk=0.9,
                               vlm_direction='center', vlm_action='turn_right'))
    assert out.state in ('AVOID_LEFT', 'AVOID_RIGHT')
    assert abs(out.angular) > 0.4
    assert out.linear <= 0.12          # 转向时低速
    assert out.risk > 0.8


def test_obstacle_on_left_turns_right():
    pl = make_planner()
    out = pl.update(free_inputs(vlm_obstacle=True, vlm_risk=0.85,
                               vlm_direction='left', vlm_action='turn_right'))
    assert out.angular < 0            # 负角速度 = 右转
    assert out.state == 'AVOID_RIGHT'


def test_depth_left_blocked_turns_right():
    # 左侧贴墙、正前方仍通畅：不应触发急避让，而是靠"侧向推离"回到中线
    pl = make_planner()
    out = pl.update(free_inputs(left=0.45, center=1.8, right=2.0,
                               vlm_ok=False, vlm_age=99.0))
    assert out.angular < 0
    assert out.state in ('CRUISE', 'CAUTION')


def test_depth_close_stops():
    pl = make_planner()
    out = pl.update(free_inputs(center=0.10, left=0.10, right=0.10,
                               vlm_ok=False, vlm_age=99.0))
    assert out.state == 'STOP'
    assert out.linear == pytest.approx(0.0)
    assert out.angular == pytest.approx(0.0)


def test_tilt_estop():
    pl = make_planner()
    out = pl.update(free_inputs(roll=1.2))
    assert out.state == 'ESTOP'
    assert out.linear == 0.0 and out.angular == 0.0


def test_stale_vlm_without_depth_stops_after_timeout():
    pl = make_planner(stop_after_stale=6.0)
    # 曾经正常工作、随后掉线：先蠕行
    out = pl.update(free_inputs(vlm_ok=False, vlm_age=4.0, sectors_valid=False,
                               sectors_age=INF, now=0.0))
    assert out.linear <= 0.13
    # 掉线超过 stop_after_stale：停
    out = pl.update(free_inputs(vlm_ok=False, vlm_age=7.0, sectors_valid=False,
                               sectors_age=INF, now=7.0))
    assert out.state == 'STOP'
    assert out.linear == pytest.approx(0.0)


def test_hold_maneuver_after_clear():
    pl = make_planner(hold_time=1.2)
    out = pl.update(free_inputs(t=0.0, vlm_obstacle=True, vlm_risk=0.9,
                               vlm_direction='center', vlm_action='turn_right'))
    turn_sign = math.copysign(1, out.angular)
    # 0.5s 后画面突然"通畅"，仍应保持一次转向
    out = pl.update(free_inputs(t=0.5, left=2.0, center=2.0, right=2.0,
                               vlm_risk=0.05))
    assert out.state in ('AVOID_LEFT', 'AVOID_RIGHT')
    # 超过 hold_time 之后应恢复巡航
    out = pl.update(free_inputs(t=2.0, left=2.0, center=2.0, right=2.0,
                               vlm_risk=0.05,
                               imu_valid=True, yaw=0.0))
    assert out.state in ('CRUISE', 'CAUTION')
    assert math.copysign(1, out.angular) != turn_sign or abs(out.angular) < 0.2


def test_heading_hold_corrects_drift():
    pl = make_planner()
    pl.update(free_inputs(t=0.0, yaw=0.0))
    out = pl.update(free_inputs(t=0.1, yaw=0.25))   # 车头偏左 0.25rad
    assert out.angular < 0                          # 应向右修正
    assert out.state in ('CRUISE', 'CAUTION')


def test_heading_relock_when_deviation_large():
    pl = make_planner()
    pl.update(free_inputs(t=0.0, yaw=0.0))
    out = pl.update(free_inputs(t=0.1, yaw=0.9))    # 偏差超过 relock 阈值
    assert abs(out.angular) <= 0.35


def test_lateral_swerve_is_bounded():
    """VLM 一直报"左侧有障碍"时，不能以恒定角速度一直右转（会绕圈）。"""
    pl = make_planner()
    kw = dict(left=0.60, right=2.50, center=2.0,
              vlm_obstacle=True, vlm_risk=0.7, vlm_direction='left',
              vlm_action='turn_right', vlm_age=0.2, vlm_latency_s=1.5,
              imu_valid=True)
    out = pl.update(free_inputs(now=0.0, yaw=0.0, **kw))
    assert out.state == 'AVOID_RIGHT', out.state
    assert out.linear > 0.2, '侧向避让应保持巡航速度'
    # 已经转过 0.9 rad（超过预算 0.8）-> 必须结束侧向避让
    out = pl.update(free_inputs(now=1.6, yaw=-0.9, **kw))
    assert out.state in ('CRUISE', 'CAUTION'), out.state


def test_lateral_claim_needs_depth_corroboration():
    """模型说"左侧有障碍"，但深度显示左侧比右侧宽 -> 不采信，不转。"""
    pl = make_planner()
    out = pl.update(free_inputs(now=0.0, left=2.50, right=0.60, center=2.0,
                                vlm_obstacle=True, vlm_risk=0.7,
                                vlm_direction='left', vlm_action='turn_right',
                                vlm_age=0.2, vlm_latency_s=1.5))
    assert out.state in ('CRUISE', 'CAUTION'), out.state
    assert abs(out.angular) < 0.3


def test_depth_hysteresis_prevents_flipflop():
    """1.0m 触发、1.3m 才解除，避免在阈值附近 CRUISE/AVOID 反复横跳。"""
    pl = make_planner()
    kw = dict(left=2.0, right=2.0, vlm_ok=False, vlm_age=99.0)
    out = pl.update(free_inputs(now=0.0, center=0.30, **kw))
    assert out.state.startswith('AVOID'), out.state
    # 退到 0.50m：超过触发距离 0.35，但还没到解除距离 0.60 -> 保持避让
    out = pl.update(free_inputs(now=3.0, center=0.50, **kw))
    assert out.state.startswith('AVOID'), out.state
    # 退到 0.70m：解除
    out = pl.update(free_inputs(now=6.0, center=0.70, **kw))
    assert out.state in ('CRUISE', 'CAUTION'), out.state


def test_recovery_after_long_stop():
    pl = make_planner(recovery_after=2.0, recovery_time=1.5)
    pl.update(free_inputs(t=0.0, center=0.20, left=0.20, right=0.20,
                          vlm_ok=False, vlm_age=99.0))
    out = pl.update(free_inputs(t=2.5, center=0.20, left=0.20, right=0.20,
                               vlm_ok=False, vlm_age=99.0))
    assert out.state == 'RECOVERY'
    assert abs(out.angular) > 0.3


def test_slow_vlm_risk_does_not_decay_away():
    """CPU 推理 16s/帧时，8s 前的障碍判断仍然必须有效。"""
    pl = make_planner()
    inp = free_inputs(vlm_obstacle=True, vlm_risk=0.9,
                      vlm_direction='center', vlm_action='turn_right',
                      vlm_age=8.0, vlm_latency_s=16.0)
    assert pl._vlm_risk(inp) > 0.4, f'慢模型的风险被衰减没了: {pl._vlm_risk(inp):.2f}'
    assert pl.update(inp).state.startswith('AVOID')


def test_fast_vlm_risk_decays_quickly():
    """1Hz 模型下，8s 前的结果应该基本失效。"""
    pl = make_planner()
    inp = free_inputs(vlm_obstacle=True, vlm_risk=0.9,
                      vlm_direction='center', vlm_action='turn_right',
                      vlm_age=8.0, vlm_latency_s=0.8)
    assert pl._vlm_risk(inp) < 0.1


def test_unknown_state_creeps_then_stops():
    pl = make_planner()
    out = pl.update(free_inputs(vlm_ok=False, vlm_age=INF, imu_valid=False,
                               sectors_valid=False, now=0.0))
    assert out.watchdog_scale == 0.0
    assert out.linear == pytest.approx(0.0)
    assert out.risk >= 0.3            # 未知风险不能当成"通"


# ---------------------------------------------------------------- IMU 滤波

def test_complementary_filter_level():
    from vlm_control.imu_filter import ComplementaryFilter
    f = ComplementaryFilter(alpha=0.98)
    for _ in range(30):
        st = f.update(0.0, 0.0, 9.81, 0.0, 0.0, 0.0, 0.01)
    assert abs(st.roll) < 0.02 and abs(st.pitch) < 0.02


def test_complementary_filter_fast_tilt_detection():
    """跌倒时必须在 ~0.2s 内识别出大倾角，否则急停来不及。"""
    import math
    from vlm_control.imu_filter import ComplementaryFilter

    roll = 1.2
    ay = -9.81 * math.sin(roll)
    az = 9.81 * math.cos(roll)
    f = ComplementaryFilter(alpha=0.98, snap_threshold=0.5)
    for _ in range(30):
        f.update(0.0, 0.0, 9.81, 0.0, 0.0, 0.0, 0.01)
    for _ in range(20):  # 0.2s
        st = f.update(0.0, ay, az, 0.0, 0.0, 0.0, 0.01)
    assert abs(st.roll) > 0.9, f'0.2s 后倾角只有 {st.roll:.2f}'


def test_imu_yaw_integrates_gyro():
    from vlm_control.imu_filter import ComplementaryFilter
    f = ComplementaryFilter(alpha=0.98)
    f.update(0.0, 0.0, 9.81, 0.0, 0.0, 0.0, 0.01)   # 初始化
    for _ in range(100):                              # 1s，1 rad/s
        st = f.update(0.0, 0.0, 9.81, 0.0, 0.0, 1.0, 0.01)
    assert st.yaw == pytest.approx(1.0, abs=0.15)
