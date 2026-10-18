"""深度扇区节点：RGB-D 深度图 -> 左/中/右最近距离（VLM 的高频安全网）。

如果你的相机是纯单目（没有深度），就不要启动本节点：
控制器会检测 /perception/sectors 是否有效，并自动退化为"仅 VLM + IMU"。
"""
from __future__ import annotations

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from vlm_avoid_interfaces.msg import SectorScan

from .depth_sectors import compute_sectors
from .image_utils import depth_msg_to_meters


class DepthSectorsNode(Node):
    def __init__(self) -> None:
        super().__init__('depth_sectors_node')
        p = self.declare_parameter
        self.depth_topic = p('depth_topic', '/camera/depth/image_raw').value
        self.result_topic = p('result_topic', '/perception/sectors/result').value
        self.max_range = float(p('max_range', 5.0).value)
        self.percentile = float(p('percentile', 10.0).value)
        self.min_valid_ratio = float(p('min_valid_ratio', 0.05).value)
        self.stride = int(p('stride', 2).value)
        # 地面过滤：相机高 0.16m、下俯 5°，vfov 由 hfov 和图像宽高比自动算
        self.camera_height = float(p('camera_height', 0.16).value)
        self.camera_pitch = float(p('camera_pitch', 0.0873).value)
        self.ground_clearance = float(p('ground_clearance', 0.06).value)
        self.horizontal_fov = float(p('horizontal_fov', 1.20).value)
        self.vertical_fov = float(p('vertical_fov', 0.0).value)
        self.left_band = tuple(float(x) for x in p('left_band', [0.0, 0.33]).value)
        self.center_band = tuple(float(x) for x in p('center_band', [0.33, 0.67]).value)
        self.right_band = tuple(float(x) for x in p('right_band', [0.67, 1.0]).value)
        self.timeout_s = float(p('timeout_s', 0.5).value)

        self._pub = self.create_publisher(SectorScan, self.result_topic, 10)
        self.create_subscription(Image, self.depth_topic, self._on_depth,
                                 qos_profile_sensor_data)
        self._last_log = 0.0
        self.get_logger().info(
            f'深度扇区节点启动: depth={self.depth_topic} -> {self.result_topic} '
            f'max_range={self.max_range}m cam_h={self.camera_height} '
            f'pitch={self.camera_pitch} '
            f'bands={self.left_band}/{self.center_band}/{self.right_band}')

    def _on_depth(self, msg: Image) -> None:
        out = SectorScan()
        out.header = msg.header
        out.header.frame_id = msg.header.frame_id or 'camera_depth_optical_frame'
        try:
            depth = depth_msg_to_meters(msg)
            # 垂直 FOV：未显式给定时按 hfov 与宽高比换算（针孔模型）
            vfov = self.vertical_fov
            if vfov <= 0.0 and depth.ndim == 2 and depth.shape[1] > 0:
                aspect = depth.shape[0] / float(depth.shape[1])
                vfov = 2.0 * math.atan(math.tan(self.horizontal_fov / 2.0) * aspect)
            s = compute_sectors(
                depth, left_band=self.left_band, center_band=self.center_band,
                right_band=self.right_band, max_range=self.max_range,
                percentile=self.percentile, min_valid_ratio=self.min_valid_ratio,
                stride=self.stride,
                camera_height=self.camera_height, camera_pitch=self.camera_pitch,
                vfov=vfov, ground_clearance=self.ground_clearance)
        except Exception as exc:  # noqa: BLE001
            out.valid = False
            out.left = out.center = out.right = out.min_range = self.max_range
            self._pub.publish(out)
            self.get_logger().error(f'深度处理失败: {exc}', throttle_duration_sec=5.0)
            return

        out.valid = bool(s['valid'])
        out.left = float(s['left'])
        out.center = float(s['center'])
        out.right = float(s['right'])
        out.min_range = float(s['min_range'])
        self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DepthSectorsNode()
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
