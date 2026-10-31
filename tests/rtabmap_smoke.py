#!/usr/bin/env python3
"""RTAB-Map 冒烟测试：不依赖 Gazebo，用合成数据验证 SLAM 能不能出 /map。

    python3 tests/rtabmap_smoke.py            # 需要先起 rtabmap_slam.launch.py

它做的事：
  1. 发静态 TF base_footprint -> camera_optical_frame / laser_link；
  2. 10Hz 发 RGB（移动的棋盘格，保证 ORB 有特征）、深度、camera_info；
  3. 10Hz 发激光（用仓库地图射线投射出来的真实回波）、IMU；
  4. 让机器人沿一条直线缓慢移动，看 RTAB-Map 是否发布 /map、TF 是否出现。
"""
from __future__ import annotations

import math
import os
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan
from tf2_ros import StaticTransformBroadcaster

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'scripts'))
sys.path.insert(0, os.path.join(ROOT, 'src/vlm_control'))

from vlm_control.grid_map import GridMap          # noqa: E402
from slam_sim import RayCaster                    # noqa: E402

W, H = 320, 240
FX, FY, CX, CY = 277.0, 277.0, 160.0, 120.0


class FakeSensors(Node):
    def __init__(self) -> None:
        super().__init__('fake_sensors')
        self.rgb = self.create_publisher(Image, '/camera_depth/image_raw', 10)
        self.depth = self.create_publisher(Image, '/camera_depth/depth/image_raw', 10)
        self.info = self.create_publisher(CameraInfo, '/camera_depth/camera_info', 10)
        self.scan = self.create_publisher(LaserScan, '/scan', 10)
        self.imu = self.create_publisher(Imu, '/imu/data', 50)
        self.wheel = self.create_publisher(Odometry, '/wheel_odom', 20)
        self._tf = StaticTransformBroadcaster(self)
        self.maps = 0
        self.create_subscription(OccupancyGrid, '/map', self._on_map, 1)
        self._send_static()

        grid = GridMap.load(os.path.join(
            ROOT, 'src/vlm_avoid_bringup/config/warehouse_map.json'))
        self.caster = RayCaster(grid, noise=0.005, n_rays=360)

        self._img = np.zeros((H, W, 3), np.uint8)
        self._depth = np.full((H, W), 2.0, np.float32)
        self._t0 = time.monotonic()
        self.create_timer(0.1, self._tick)

    def _send_static(self) -> None:
        for child, xyz in ((('camera_optical_frame'), (0.12, 0.0, 0.30)),
                           (('laser_link'), (0.0, 0.0, 0.21))):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'base_footprint'
            t.child_frame_id = child
            t.transform.translation.x, t.transform.translation.y, \
                t.transform.translation.z = xyz
            t.transform.rotation.w = 1.0
            self._tf.sendTransform(t)

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.maps += 1
        if self.maps in (1, 5):
            self.get_logger().info(
                f'收到 /map: {msg.info.width}x{msg.info.height} '
                f'@{msg.info.resolution:.2f}m frame={msg.header.frame_id}')

    def _tick(self) -> None:
        self._send_static()          # 每次都发：静态 TF 是锁存的，重复发无害
        now = time.monotonic() - self._t0
        x = 0.15 * now          # 缓慢直行
        yaw = 0.0
        stamp = self.get_clock().now().to_msg()

        # 移动的纹理（保证视觉有特征）
        img = np.zeros((H, W, 3), np.uint8)
        shift = int(now * 20) % W
        for i in range(0, W, 24):
            col = 255 if (i // 24) % 2 else 90
            img[:, (i + shift) % W:(i + shift) % W + 12] = col
        img[:, :4] = 255
        self._img = img

        m = Image()
        m.header.stamp, m.header.frame_id = stamp, 'camera_optical_frame'
        m.height, m.width, m.encoding, m.step = H, W, 'bgr8', W * 3
        m.data = self._img.tobytes()
        self.rgb.publish(m)

        d = Image()
        d.header.stamp, d.header.frame_id = stamp, 'camera_optical_frame'
        d.height, d.width, d.encoding, d.step = H, W, '32FC1', W * 4
        d.data = self._depth.tobytes()
        self.depth.publish(d)

        ci = CameraInfo()
        ci.header.stamp, ci.header.frame_id = stamp, 'camera_optical_frame'
        ci.width, ci.height = W, H
        ci.k = [FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0]
        ci.p = [FX, 0.0, CX, 0.0, 0.0, FY, CY, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.info.publish(ci)

        od = Odometry()
        od.header.stamp, od.header.frame_id = stamp, 'odom'
        od.child_frame_id = 'base_footprint'
        od.pose.pose.position.x = x
        od.pose.pose.orientation.w = 1.0
        self.wheel.publish(od)

        ranges = self.caster.cast((1.5 + x, 1.5, yaw))
        ls = LaserScan()
        ls.header.stamp, ls.header.frame_id = stamp, 'laser_link'
        ls.angle_min = -math.pi
        ls.angle_max = math.pi
        ls.angle_increment = 2 * math.pi / len(ranges)
        ls.range_min, ls.range_max = 0.12, 12.0
        ls.ranges = ranges.astype(np.float32).tolist()
        self.scan.publish(ls)


def main() -> int:
    rclpy.init()
    node = FakeSensors()
    t_end = time.monotonic() + 40.0
    while time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)
    print(f'/map 消息数: {node.maps}')
    node.destroy_node()
    rclpy.shutdown()
    return 0 if node.maps > 0 else 1


if __name__ == '__main__':
    sys.exit(main())
