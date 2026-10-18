#!/usr/bin/env python3
"""为什么车不动？—— 一键诊断。

在仿真/真机跑着的时候，另开一个终端执行：

    python3 scripts/doctor.py

它会读 5 秒，然后逐项告诉你：
  * Gazebo 是否在推进（/clock 有没有走）、是否被暂停
  * 相机 / 深度 / IMU / odom 有没有数据、多少 Hz
  * VLM 是否返回、延迟多少、有没有 stale
  * 控制器在什么状态、有没有被 enable=false 或 require_imu 拦住
  * 最后给出一句结论：到底是"感知没进来"、"底盘不响应"还是"正常在避障"
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
from nav_msgs.msg import Odometry  # noqa: E402
from rcl_interfaces.msg import ParameterType  # noqa: E402
from rcl_interfaces.srv import GetParameters, SetParameters  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import (QoSProfile, ReliabilityPolicy,  # noqa: E402
                       qos_profile_sensor_data)
from rosgraph_msgs.msg import Clock  # noqa: E402
from sensor_msgs.msg import Image, Imu  # noqa: E402
from vlm_avoid_interfaces.msg import (DriveStatus, SectorScan,  # noqa: E402
                                      VlmObstacle)

TYPES = {'bool': ParameterType.PARAMETER_BOOL,
         'double': ParameterType.PARAMETER_DOUBLE,
         'int': ParameterType.PARAMETER_INTEGER,
         'str': ParameterType.PARAMETER_STRING}


class Doctor(Node):
    def __init__(self) -> None:
        super().__init__('doctor')
        self.n = {}
        self.last = {}
        self.first_t = time.monotonic()
        self.clock_first = None
        self.clock_last = None
        self.odom_first = None
        self.odom_last = None

        def bump(key):
            self.n[key] = self.n.get(key, 0) + 1

        # Gazebo 的 /clock 是 BEST_EFFORT，用默认 reliable 会收不到
        clock_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Clock, '/clock', self._on_clock, clock_qos)
        self.create_subscription(Image, '/camera/image_raw',
                                 lambda m: bump('image'), qos_profile_sensor_data)
        # 深度图有两个可能的合法名字：真机用 canonical 的 /camera/depth/...，
        # Gazebo 插件会拼成 /camera_depth/depth/...（camera_name 前缀）
        for depth_topic in ('/camera/depth/image_raw', '/camera_depth/depth/image_raw'):
            self.create_subscription(Image, depth_topic,
                                     lambda m: bump('depth'), qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu/data', lambda m: bump('imu'),
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(Twist, '/cmd_vel',
                                 lambda m: (bump('cmd'), self.last.__setitem__('cmd', m)), 10)
        self.create_subscription(VlmObstacle, '/perception/vlm/result',
                                 lambda m: (bump('vlm'), self.last.__setitem__('vlm', m)), 10)
        self.create_subscription(SectorScan, '/perception/sectors/result',
                                 lambda m: (bump('sec'), self.last.__setitem__('sec', m)),
                                 qos_profile_sensor_data)
        self.create_subscription(DriveStatus, '/vlm_avoid/status',
                                 lambda m: (bump('status'), self.last.__setitem__('status', m)), 10)

    def _on_clock(self, msg: Clock):
        t = msg.clock.sec + msg.clock.nanosec * 1e-9
        if self.clock_first is None:
            self.clock_first = t
        self.clock_last = t
        self.n['clock'] = self.n.get('clock', 0) + 1

    def _on_odom(self, msg: Odometry):
        self.n['odom'] = self.n.get('odom', 0) + 1
        self.last['odom'] = msg
        p = msg.pose.pose.position
        if self.odom_first is None:
            self.odom_first = (p.x, p.y)
        self.odom_last = (p.x, p.y)

    # ---- 参数读写 ----
    def get_params(self, node_name, names):
        cli = self.create_client(GetParameters, f'/{node_name}/get_parameters')
        if not cli.wait_for_service(timeout_sec=10.0):
            return None
        from rcl_interfaces.srv import GetParameters as GP
        req = GP.Request()
        req.names = names
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        if fut.result() is None:
            return None
        out = {}
        for name, v in zip(names, fut.result().values):
            out[name] = (v.bool_value if v.type == 1 else
                         v.double_value if v.type == 3 else
                         v.integer_value if v.type == 2 else v.string_value)
        return out

    def set_bool(self, node_name, name, value):
        cli = self.create_client(SetParameters, f'/{node_name}/set_parameters')
        if not cli.wait_for_service(timeout_sec=10.0):
            print(f'  ! 找不到 /{node_name} 的参数服务（控制器没启动？）')
            return False
        from rcl_interfaces.srv import SetParameters as SP
        from rcl_interfaces.msg import Parameter, ParameterValue
        req = SP.Request(parameters=[Parameter(
            name=name,
            value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value))])
        for attempt in range(2):
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
            res = fut.result()
            if res is not None:
                ok = bool(res.results[0].successful)
                if not ok:
                    print(f'  ! 被拒绝: {res.results[0].reason}')
                return ok
            print(f'  ! 参数服务无响应，重试 {attempt + 1}/2')
        return False


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=5.0)
    ap.add_argument('--wait-vlm', type=float, default=20.0,
                    help='没有 VLM 结果时最多再等多久（CPU 推理一帧要十几秒）')
    ap.add_argument('--resume', action='store_true',
                    help='把控制器 enable 设回 true（车不动时试试）')
    ap.add_argument('--disable', action='store_true',
                    help='把控制器 enable 设成 false（想手动 teleop 时用）')
    args = ap.parse_args()

    rclpy.init()
    d = Doctor()
    if args.resume or args.disable:
        ok = d.set_bool('avoidance_controller', 'enable', args.resume)
        print(f'{"✓ 已恢复" if args.resume else "✓ 已关闭"}控制器 '
              f'(enable={args.resume}, success={ok})')
        d.destroy_node()
        rclpy.shutdown()
        return 0 if ok else 1
    t0 = time.monotonic()
    while time.monotonic() - t0 < args.seconds:
        rclpy.spin_once(d, timeout_sec=0.05)
    dt = max(1e-3, time.monotonic() - t0)

    # 没有 VLM 结果时再等一会：CPU 推理单帧 15s 很常见，5s 窗口会误判成"没结果"
    if d.last.get('vlm') is None and args.wait_vlm > 0 and not (args.resume or args.disable):
        print(f'等待首帧 VLM 结果（最多 {args.wait_vlm:.0f}s）...')
        t_wait = time.monotonic()
        while time.monotonic() - t_wait < args.wait_vlm:
            rclpy.spin_once(d, timeout_sec=0.1)
            if d.last.get('vlm') is not None:
                print(f'  ✓ {time.monotonic() - t_wait:.1f}s 后收到首帧')
                break
        else:
            print('  ✗ 仍然没有结果')

    def hz(key):
        return d.n.get(key, 0) / dt

    print(f'===== 诊断（采样 {dt:.1f}s）=====')
    print('话题：')
    rows = [('相机 /camera/image_raw', 'image', 3.0),
            ('深度 /camera/depth/image_raw', 'depth', 3.0),
            ('IMU  /imu/data', 'imu', 15.0),
            ('里程 /odom', 'odom', 5.0),
            ('指令 /cmd_vel', 'cmd', 10.0),
            ('VLM  /perception/vlm/result', 'vlm', 0.1),
            ('扇区 /perception/sectors/result', 'sec', 2.0),
            ('状态 /vlm_avoid/status', 'status', 5.0)]
    for label, key, want in rows:
        got = hz(key)
        mark = '✓' if got >= want else ('~' if got > 0 else '✗')
        print(f'  {mark} {label:<34} {got:6.1f} Hz')

    sim = hz('clock') > 0.5
    if sim:
        adv = (d.clock_last or 0) - (d.clock_first or 0)
        print(f'  {"✓" if adv > 0.1 else "!"} Gazebo /clock 推进了 {adv:.2f}s'
              + ('' if adv > 0.1 else '  ← 可能被暂停 pause:=true'))
    else:
        print('  · 没有 /clock（真机模式或 Gazebo 没起）')

    params = d.get_params('avoidance_controller', ['enable', 'require_imu'])
    if params is None:
        print('  ✗ 读不到 /avoidance_controller 参数（控制器没起？）')
    else:
        print(f'  控制器参数: enable={params.get("enable")} '
              f'require_imu={params.get("require_imu")}')

    st = d.last.get('status')
    cmd = d.last.get('cmd')
    vlm = d.last.get('vlm')
    sec = d.last.get('sec')
    odom_moved = (d.odom_first is not None and d.odom_last is not None and
                  ((d.odom_last[0] - d.odom_first[0]) ** 2 +
                   (d.odom_last[1] - d.odom_first[1]) ** 2) ** 0.5)
    print('\n当前值：')
    if st:
        print(f'  状态={st.state} 风险={st.risk:.2f} 净空={st.clearance_center:.2f}m '
              f'vlm_stale={st.vlm_stale}')
    if cmd:
        print(f'  cmd_vel: v={cmd.linear.x:+.2f} w={cmd.angular.z:+.2f}')
    if vlm:
        print(f'  VLM: {"障碍" if vlm.obstacle else "通畅"} risk={vlm.risk:.2f} '
              f'{vlm.direction}/{vlm.action} {vlm.latency_ms:.0f}ms'
              f'{" STALE" if vlm.stale else ""} label={vlm.label!r}')
    else:
        print('  VLM: 没有任何结果（模型慢/Ollama 没起都会这样）')
        print('       检查: bash scripts/ollama_serve.sh && bash scripts/ollama.sh list')
    if sec:
        print(f'  扇区: valid={sec.valid} L={sec.left:.2f} C={sec.center:.2f} R={sec.right:.2f}')
    else:
        print('  扇区: 没有数据')
    print(f'  里程位移: {odom_moved:.3f} m')

    print('\n图像话题（找相机用，publishers>0 才说明插件在发）:')
    found = False
    for topic, types in sorted(d.get_topic_names_and_types()):
        if any('sensor_msgs/msg/Image' in t for t in types) or topic.endswith('/points'):
            print(f'  {topic:<45} publishers={d.count_publishers(topic)}')
            found = True
    if not found:
        print('  （一个 Image 话题都没有 → 相机插件没加载成功）')

    # ---------------- 结论 ----------------
    print('\n===== 结论 =====')
    if hz('cmd') < 1.0:
        print('  控制器没在发 /cmd_vel。检查 /avoidance_controller 是否在跑、'
              'enable 是否为 true。')
        return 1
    if params and params.get('enable') is False:
        print('  控制器被 enable=false 关掉了。恢复：')
        print('    python3 scripts/doctor.py --resume')
        return 1
    if st and st.state == 'ESTOP':
        print('  IMU 倾角超限导致 ESTOP（车翻了或 IMU 姿态异常）。')
        return 1
    if hz('image') < 1.0 and hz('depth') < 1.0 and (vlm is None or vlm.stale):
        print('  ✗ 感知完全没进来（相机没图、深度没图、VLM 无结果）→ 控制器按安全策略停车。')
        print('    常见原因：Gazebo 相机插件渲染失败、相机话题名不对、'
              'use_sim_time/ROS_DOMAIN_ID 不一致。')
        print('    先看: ros2 topic hz /camera/image_raw（有图就会有 ~10Hz）')
        return 1
    if cmd and abs(cmd.linear.x) < 1e-3 and abs(cmd.angular.z) < 1e-3:
        if st and st.state == 'STOP':
            print('  控制器主动 STOP（前方净空不足或 VLM 报高风险）。')
            print(f'    看 /perception/vlm/annotated 或 /vlm_avoid/status 里的 risk/clearance。')
        else:
            print('  指令为 0 但状态不是 STOP，检查 enable 与规划参数。')
        return 1
    if hz('odom') < 0.5:
        print('  · 没有 /odom：只有 Gazebo/真机底盘才会发里程。')
        print('    决策链路本身是通的（/cmd_vel 在输出），但"车会不会动"这里验证不了。')
        print('    若你确实在 Gazebo 里跑却看不到 /odom → 差速插件没加载。')
        return 0
    if odom_moved < 0.01:
        print('  ✗ 有速度指令但里程没动 → 底盘没响应。检查：')
        print('    * Gazebo 是否暂停（上面 /clock 是否推进）')
        print('    * 差速插件是否加载: grep diff_drive gazebo 日志')
        print('    * 轮子是否悬空/穿地（spawn 高度 -z）')
        return 1
    print(f'  ✓ 一切正常：state={st.state if st else "-"}，'
          f'指令 v={cmd.linear.x:+.2f}，{dt:.1f}s 内位移 {odom_moved:.3f}m。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
