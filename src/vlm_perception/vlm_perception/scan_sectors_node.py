"""激光雷达 → 三扇区节点。

订阅 sensor_msgs/LaserScan（默认 /scan），发布与深度通道同构的 SectorScan
（默认 /perception/scan_sectors），控制器会把它和深度扇区逐项取最小值。
"""
from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from vlm_avoid_interfaces.msg import SectorScan

from .scan_sectors import compute_scan_sectors


class ScanSectorsNode(Node):
    def __init__(self) -> None:
        super().__init__('scan_sectors_node')
        p = self.declare_parameter
        self.scan_topic = p('scan_topic', '/scan').value
        self.result_topic = p('result_topic', '/perception/scan_sectors').value
        self.front_angle_deg = float(p('front_angle_deg', 40.0).value)
        self.side_angle_deg = float(p('side_angle_deg', 100.0).value)
        self.max_range = float(p('max_range', 12.0).value)
        self.range_min = float(p('range_min', 0.12).value)
        self.range_max = float(p('range_max', 20.0).value)

        self._front_half = math.radians(self.front_angle_deg) / 2.0
        self._side = math.radians(self.side_angle_deg)

        self._pub = self.create_publisher(SectorScan, self.result_topic, 10)
        self.create_subscription(LaserScan, self.scan_topic, self._on_scan,
                                 qos_profile_sensor_data)
        self.get_logger().info(
            f'雷达扇区节点启动: {self.scan_topic} -> {self.result_topic} '
            f'front=±{self.front_angle_deg / 2:.0f}° side=±{self.side_angle_deg:.0f}° '
            f'max_range={self.max_range}m')

    def _on_scan(self, msg: LaserScan) -> None:
        out = SectorScan()
        out.header = msg.header
        out.header.frame_id = msg.header.frame_id or 'laser_link'
        try:
            s = compute_scan_sectors(
                msg.ranges, msg.angle_min, msg.angle_increment,
                range_min=max(self.range_min, float(msg.range_min)),
                range_max=min(self.range_max, float(msg.range_max)),
                front_half_angle=self._front_half, side_angle=self._side,
                max_range=self.max_range)
        except Exception as exc:  # noqa: BLE001
            out.valid = False
            out.left = out.center = out.right = out.min_range = self.max_range
            self._pub.publish(out)
            self.get_logger().error(f'雷达处理失败: {exc}', throttle_duration_sec=5.0)
            return

        out.valid = bool(s['valid'])
        out.left = float(s['left'])
        out.center = float(s['center'])
        out.right = float(s['right'])
        out.min_range = float(s['min_range'])
        self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanSectorsNode()
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
