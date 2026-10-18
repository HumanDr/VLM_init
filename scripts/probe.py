#!/usr/bin/env python3
"""实时观察避障管线：状态机 / 风险 / 净空 / 速度 / VLM 判断。

    python3 scripts/probe.py                 # 每 2 秒一行，Ctrl-C 退出
    python3 scripts/probe.py --interval 0.5
    python3 scripts/probe.py --duration 60

真模型（尤其 CPU 推理）单帧可能十几秒，用这个比 ros2 topic echo 好看清楚。
"""
from __future__ import annotations

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ros_bootstrap import ensure_env  # noqa: E402

ensure_env()

import rclpy  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from rclpy.node import Node  # noqa: E402
from vlm_avoid_interfaces.msg import DriveStatus, VlmObstacle  # noqa: E402


class Probe(Node):
    def __init__(self) -> None:
        super().__init__('vlm_avoid_probe')
        self.status = None
        self.cmd = None
        self.vlm = None
        self.vlm_t = 0.0
        self.create_subscription(DriveStatus, '/vlm_avoid/status', self._on_status, 10)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)
        self.create_subscription(VlmObstacle, '/perception/vlm/result', self._on_vlm, 10)

    def _on_status(self, msg):
        self.status = msg

    def _on_cmd(self, msg):
        self.cmd = msg

    def _on_vlm(self, msg):
        self.vlm = msg
        self.vlm_t = time.monotonic()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--interval', type=float, default=2.0)
    ap.add_argument('--duration', type=float, default=0.0, help='0 = 一直跑')
    args = ap.parse_args()

    rclpy.init()
    node = Probe()
    print('时间    状态       风险  净空   v(m/s)  w(rad/s)  VLM判断')
    print('-' * 92)
    empty_since = time.monotonic()
    hinted = False
    t0 = time.monotonic()
    t_start = t0
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() - t0 >= args.interval:
                t0 = time.monotonic()
                s, c, v = node.status, node.cmd, node.vlm
                age = (time.monotonic() - node.vlm_t) if node.vlm else -1
                vtxt = '-'
                if v is not None:
                    vtxt = (f"{'障碍' if v.obstacle else '通畅'} risk={v.risk:.2f} "
                            f"{v.direction}/{v.action} {v.latency_ms:.0f}ms"
                            f"{' STALE' if v.stale else ''} age={age:.0f}s")
                print(f'{time.strftime("%H:%M:%S")} '
                      f'{(s.state if s else "-"):<10} '
                      f'{(s.risk if s else 0):.2f}  '
                      f'{(s.clearance_center if s else 0):.2f}  '
                      f'{(c.linear.x if c else 0):+.2f}   '
                      f'{(c.angular.z if c else 0):+.2f}    {vtxt}')
                if node.status is None and not hinted and time.monotonic() - empty_since > 6:
                    hinted = True
                    print('  (还没收到 /vlm_avoid/status —— 管线没启动？'
                          '或 ROS_DOMAIN_ID 不一致？ 用 ros2 topic list 确认)')
                elif node.status is not None:
                    empty_since = time.monotonic()
                if args.duration and time.monotonic() - t_start >= args.duration:
                    break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
