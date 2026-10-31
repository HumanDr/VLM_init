#!/usr/bin/env python3
"""无 Gazebo 的端到端联调测试。

链路：假相机(图像+深度+IMU) -> VLM 感知节点 -> Mock VLM HTTP 服务
      -> 深度扇区 -> 避障控制器 -> /cmd_vel

它会真实启动 launch 文件与子进程，验证：
  1. 通畅时前进；
  2. 正前方出现障碍时转向避让且降速；
  3. 视野恢复后回到巡航；
  4. IMU 姿态超限时急停；
  5. VLM 服务挂掉后标记 stale 并降级，而不是继续满速冲。

用法：
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    python3 tests/integration_mock.py            # 需要先 colcon build
"""
from __future__ import annotations

import argparse
import math
import os
import signal
import socket
import subprocess
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), 'scripts'))
from ros_bootstrap import ensure_env  # noqa: E402

ensure_env()

import rclpy  # noqa: E402
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vlm_avoid_interfaces.msg import DriveStatus, SectorScan


def free_port() -> int:
    with socket.socket() as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


class Probe(Node):
    def __init__(self) -> None:
        super().__init__('integration_probe')
        self.cmds: list[tuple[float, float, float]] = []
        self.states: list[tuple[float, str, float, bool]] = []
        self.sectors: list[tuple[float, float, float, float, bool]] = []
        self.scan_sectors: list[tuple[float, float, float, float, bool]] = []
        self.plans: list[tuple[float, list]] = []
        self.annotated = 0
        self._pub_scene = self.create_publisher(String, '/test/scene', 10)
        self._pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self._pub_goal = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)
        self.create_subscription(DriveStatus, '/vlm_avoid/status', self._on_status, 10)
        self.create_subscription(SectorScan, '/perception/sectors/result',
                                 self._on_sector, 10)
        self.create_subscription(
            Path, '/plan',
            lambda m: self.plans.append(
                (time.monotonic(),
                 [(p.pose.position.x, p.pose.position.y) for p in m.poses])), 10)
        self.create_subscription(SectorScan, '/perception/scan_sectors',
                                 self._on_scan_sector, 10)
        self.create_subscription(Image, '/perception/vlm/annotated',
                                 self._on_image, 2)

    def _on_cmd(self, msg: Twist) -> None:
        self.cmds.append((time.monotonic(), msg.linear.x, msg.angular.z))

    def _on_status(self, msg: DriveStatus) -> None:
        self.states.append((time.monotonic(), msg.state, msg.risk, msg.vlm_stale,
                            msg.distance_to_goal))

    def _on_sector(self, msg: SectorScan) -> None:
        self.sectors.append((time.monotonic(), msg.center, msg.left, msg.right,
                             msg.valid))

    def _on_scan_sector(self, msg: SectorScan) -> None:
        self.scan_sectors.append((time.monotonic(), msg.center, msg.left, msg.right,
                                  msg.valid))

    def _on_image(self, _msg: Image) -> None:
        self.annotated += 1

    def set_scene(self, scene: str) -> None:
        self._pub_scene.publish(String(data=scene))

    def publish_odom(self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0) -> None:
        """假里程计：控制器靠它算目标距离"""
        m = Odometry()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'odom'
        m.child_frame_id = 'base_footprint'
        m.pose.pose.position.x = float(x)
        m.pose.pose.position.y = float(y)
        m.pose.pose.orientation.z = math.sin(yaw / 2.0)
        m.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self._pub_odom.publish(m)

    def send_goal(self, x: float, y: float) -> None:
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'odom'
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.orientation.w = 1.0
        self._pub_goal.publish(m)


def spin_for(node: Node, probe: Probe, seconds: float, scene: str | None,
             odom: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> None:
    t_end = time.monotonic() + seconds
    while time.monotonic() < t_end:
        if scene is not None:
            probe.set_scene(scene)
        probe.publish_odom(*odom)
        rclpy.spin_once(probe, timeout_sec=0.02)


def wait_for(probe: Probe, predicate, timeout: float, what: str) -> bool:
    t_end = time.monotonic() + timeout
    while time.monotonic() < t_end:
        rclpy.spin_once(probe, timeout_sec=0.05)
        if predicate():
            return True
    print(f'  ✗ 等待超时: {what} ({timeout:.0f}s)')
    return False


def window(probe: Probe, t0: float, t1: float):
    cmds = [c for c in probe.cmds if t0 <= c[0] <= t1]
    states = [s for s in probe.states if t0 <= s[0] <= t1]
    return cmds, states


def start(cmd: list[str], log_path: str, env: dict) -> subprocess.Popen:
    log = open(log_path, 'w', encoding='utf-8')
    log.write(f'# {" ".join(cmd)}\n')
    log.flush()
    return subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                            start_new_session=True, env=env)


def stop(proc: subprocess.Popen | None, name: str) -> None:
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=8)
    except Exception:  # noqa: BLE001
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception:  # noqa: BLE001
            pass


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--logs-dir', default=os.path.join(os.getcwd(), '.dsh_test_logs'))
    ap.add_argument('--keep', action='store_true', help='结束后保留进程')
    args = ap.parse_args(argv)

    os.makedirs(args.logs_dir, exist_ok=True)
    env = dict(os.environ)
    env.setdefault('ROS_LOG_DIR', os.path.join(os.getcwd(), '.dsh_ros_log'))
    env.setdefault('ROS_HOME', os.path.join(os.getcwd(), '.dsh_ros_home'))
    port = free_port()
    url = f'http://127.0.0.1:{port}'

    procs: dict[str, subprocess.Popen] = {}
    rclpy.init()
    node = rclpy.create_node('integration_driver')
    probe = Probe()
    failures: list[str] = []

    def check(cond: bool, ok_msg: str, fail_msg: str) -> None:
        if cond:
            print(f'  ✓ {ok_msg}')
        else:
            print(f'  ✗ {fail_msg}')
            failures.append(fail_msg)

    try:
        print('[1/6] 启动 Mock VLM / 假相机 / 避障管线 ...')
        procs['mock'] = start([sys.executable, '-m', 'vlm_perception.mock_vlm_server',
                               '--port', str(port)],
                              os.path.join(args.logs_dir, 'mock_vlm.log'), env)
        procs['camera'] = start(['ros2', 'run', 'vlm_perception', 'fake_camera'],
                                os.path.join(args.logs_dir, 'fake_camera.log'), env)
        procs['pipeline'] = start(
            ['ros2', 'launch', 'vlm_avoid_bringup', 'avoidance.launch.py',
             'use_sim_time:=false', f'vlm_url:={url}', 'vlm_model:=mock-vlm',
             'enable_depth:=true', 'publish_debug_image:=true',
             'rviz:=false'],
            os.path.join(args.logs_dir, 'pipeline.log'), env)

        if not wait_for(probe, lambda: probe.count_publishers('/cmd_vel') > 0, 25,
                        '/cmd_vel 发布者'):
            return 1
        spin_for(node, probe, 3.0, 'free')

        print('[2/6] 通畅场景：应前进')
        t0 = time.monotonic()
        spin_for(node, probe, 3.0, 'free')
        cmds, states = window(probe, t0, time.monotonic())
        v_med = sorted(c[1] for c in cmds)[len(cmds) // 2] if cmds else 0.0
        check(v_med > 0.05, f'前进 v={v_med:.2f} m/s（{len(cmds)} 帧）',
              f'通畅时没有前进 v={v_med:.2f}')
        check(any(s[1] in ('CRUISE', 'CAUTION') for s in states),
              '状态处于 CRUISE/CAUTION',
              f'状态异常: {sorted({s[1] for s in states})}')

        print('[3/6] 正前方障碍：应转向避让并降速')
        t0 = time.monotonic()
        spin_for(node, probe, 5.0, 'center')
        # 只看最后 2 秒（决策+平滑收敛后的稳态），排除切换瞬间
        cmds, states = window(probe, t0 + 3.0, time.monotonic())
        avoid = [s for s in states if s[1].startswith('AVOID')]
        check(bool(avoid), f'进入避让状态 {sorted({s[1] for s in states})}',
              f'未进入避让状态: {sorted({s[1] for s in states})}')
        w = [c[2] for c in cmds]
        check(bool(w) and max((abs(x) for x in w), default=0.0) > 0.3,
              f'产生转向角速度 max|w|={max(abs(x) for x in w):.2f}',
              '没有转向角速度')
        check(bool(cmds) and max((c[1] for c in cmds), default=9.9) < 0.15,
              f'避让时降速 max v={max(c[1] for c in cmds):.2f}',
              f'避让时速度过高 max v={max(c[1] for c in cmds):.2f}')
        recent = [s for s in probe.sectors if s[0] >= t0 and s[4]]
        check(bool(recent) and any(abs(s[1] - 0.30) < 0.15 for s in recent),
              '深度扇区识别出前方 ~0.30m 障碍',
              f'深度扇区异常: {[(round(s[1], 2)) for s in recent[:5]]}')

        recent_scan = [s for s in probe.scan_sectors if s[0] >= t0 and s[4]]
        check(bool(recent_scan) and any(abs(s[1] - 0.30) < 0.15 for s in recent_scan),
              '激光雷达扇区同样识别出 ~0.30m 障碍',
              f'雷达扇区异常: {[(round(s[1], 2)) for s in recent_scan[:5]]}')

        print('[4/6] 视野恢复：应回到巡航')
        t0 = time.monotonic()
        spin_for(node, probe, 4.0, 'free')
        cmds, states = window(probe, t0, time.monotonic())
        check(any(s[1] in ('CRUISE', 'CAUTION') for s in states),
              '恢复巡航状态', f'未恢复: {sorted({s[1] for s in states})}')
        check(bool(cmds) and max((c[1] for c in cmds), default=0.0) > 0.15,
              '恢复前进', '恢复后仍未前进')

        print('[5/6] IMU 倾覆：应急停')
        t0 = time.monotonic()
        spin_for(node, probe, 4.0, 'tilt')
        cmds, states = window(probe, t0, time.monotonic())
        check(any(s[1] == 'ESTOP' for s in states), '进入 ESTOP',
              f'未急停: {sorted({s[1] for s in states})}')
        tail = [c for c in cmds if c[0] > t0 + 2.0]
        check(bool(tail) and max((abs(c[1]) + abs(c[2]) for c in tail), default=9.9) < 1e-6,
              'ESTOP 后速度指令归零', 'ESTOP 后仍有速度指令')

        print('[6/6] VLM 服务挂掉：应标记 stale 并降级')
        spin_for(node, probe, 1.0, 'free')
        stop(procs['mock'], 'mock')
        procs.pop('mock', None)
        t0 = time.monotonic()
        spin_for(node, probe, 9.0, 'free')
        cmds, states = window(probe, t0 + 4.0, time.monotonic())
        check(any(s[3] for s in states), 'status 中 vlm_stale=True',
              '未检测到 stale 标记')
        check(bool(cmds) and max((c[1] for c in cmds), default=9.9) <= 0.52,   # 巡航 0.70 的 75%
              f'降级后限速 v<={max((c[1] for c in cmds), default=0):.2f}',
              f'降级后仍在满速: {max((c[1] for c in cmds), default=0):.2f}')
        check(probe.annotated > 0, f'调试叠加图已发布（{probe.annotated} 帧）',
              '没有收到 annotated 图像')

        print('[7/7] 目的地导航：给一个左侧目标，应转向并进入 GOTO')
        probe.send_goal(0.0, 3.0)          # 起点 (0,0) 朝 +x，目标在左前方 90°
        t0 = time.monotonic()
        spin_for(node, probe, 4.0, 'free', odom=(0.0, 0.0, 0.0))
        cmds, states = window(probe, t0 + 1.0, time.monotonic())
        states_txt = sorted({s[1] for s in states})
        check('GOTO' in states_txt, f'进入 GOTO 状态 {states_txt}',
              f'未进入 GOTO: {states_txt}')
        w = [c[2] for c in cmds]
        check(bool(w) and max(w) > 0.5, f'朝目标左转 w={max(w, default=0):+.2f}',
              f'转向不正确: {w[:5]}')
        st = probe.states[-1] if probe.states else None
        check(st is not None and 2.5 < st[4] < 3.5,
              f'距离目标 {st[4]:.2f} m' if st else '无状态',
              'distance_to_goal 不正确')
        # 再给一个"就在脚下"的目标，应进入 ARRIVED 并停车
        probe.send_goal(0.1, 0.0)
        t0 = time.monotonic()
        spin_for(node, probe, 3.0, 'free', odom=(0.0, 0.0, 0.0))
        cmds, states = window(probe, t0 + 1.5, time.monotonic())
        check(any(s[1] == 'ARRIVED' for s in states), '到达目标后进入 ARRIVED',
              f'未到达: {sorted({s[1] for s in states})}')
        check(bool(cmds) and max(abs(c[1]) + abs(c[2]) for c in cmds) < 1e-6,
              'ARRIVED 后速度归零', 'ARRIVED 后仍有速度')

        print('[8/8] A* 绕墙：目标在墙 A 后面（odom 20.5,0 -> 世界 22,1.5）')
        t0 = time.monotonic()
        probe.send_goal(20.5, 0.0)
        spin_for(node, probe, 6.0, 'free', odom=(0.0, 0.0, 0.0))
        plans = [p for p in probe.plans if p[0] >= t0]
        n_wp = len(plans[-1][1]) if plans else 0
        check(n_wp >= 3, f'收到 A* 路径（{n_wp} 个路点）', '没有收到 /plan')
        north = max((y for _t, pts in plans for _x, y in pts), default=-99.0)
        check(north > 0.5, f'路径向北绕行（最北 y={north:.2f}m）',
              f'路径没有绕墙（最北 y={north:.2f}）')
        cmds, states = window(probe, t0 + 1.0, time.monotonic())
        check('GOTO' in {s[1] for s in states},
              '处于 GOTO 状态', f'状态异常: {sorted({s[1] for s in states})}')
        w = [c[2] for c in cmds]
        # 目标正东（直线航向 0），而路径先向东北 -> 角速度应为正但不大（斜切约 0.11 rad）
        check(bool(w) and max(w) > 0.08,
              f'朝第一段路点转向（w={max(w, default=0):+.2f}，直线目标本应为 0）',
              f'没有朝路点转向: w={w[:5]}')
        check(bool(w) and min(w) > -0.05,
              '全程没有朝反方向打舵', f'出现反向角速度: {min(w, default=0):+.2f}')

    finally:
        if args.keep:
            print(f'保留进程：{ {k: p.pid for k, p in procs.items()} }')
        else:
            for name, proc in procs.items():
                stop(proc, name)
        node.destroy_node()
        rclpy.shutdown()

    print()
    if failures:
        print(f'FAILED: {len(failures)} 项')
        for f in failures:
            print(f'  - {f}')
        print(f'日志目录: {args.logs_dir}')
        return 1
    print('PASSED: 全链路（相机->VLM->决策->cmd_vel）行为符合预期')
    return 0


if __name__ == '__main__':
    sys.exit(main())
