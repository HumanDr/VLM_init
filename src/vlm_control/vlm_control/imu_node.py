"""IMU 状态估计节点：sensor_msgs/Imu -> ImuState(roll/pitch/yaw)。

刻意做成独立节点：真机上换 EKF（robot_localization）时，控制器完全不用改。
"""
from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from vlm_avoid_interfaces.msg import ImuState

from .imu_filter import ComplementaryFilter
from .planner import clamp


class ImuStateNode(Node):
    def __init__(self) -> None:
        super().__init__('imu_state_node')
        p = self.declare_parameter
        self.imu_topic = p('imu_topic', '/imu/data').value
        self.state_topic = p('state_topic', '/state/imu').value
        self.alpha = float(p('alpha', 0.98).value)
        self.snap_threshold = float(p('snap_threshold', 0.5).value)
        self.rate_hz = float(p('rate_hz', 30.0).value)
        self.timeout = float(p('timeout', 0.5).value)

        self._filter = ComplementaryFilter(alpha=self.alpha,
                                           snap_threshold=self.snap_threshold)
        self._raw = None
        self._last_t = None
        self._last_msg_t = 0.0

        self._pub = self.create_publisher(ImuState, self.state_topic, 10)
        self.create_subscription(Imu, self.imu_topic, self._on_imu,
                                 qos_profile_sensor_data)
        self.create_timer(1.0 / max(1.0, self.rate_hz), self._tick)
        self.get_logger().info(
            f'IMU 状态估计启动: {self.imu_topic} -> {self.state_topic} '
            f'alpha={self.alpha} snap={self.snap_threshold} {self.rate_hz:.0f}Hz')

    def _on_imu(self, msg: Imu) -> None:
        self._raw = msg
        self._last_msg_t = time.monotonic()

    def _tick(self) -> None:
        now = time.monotonic()
        out = ImuState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'imu_link'
        valid = (self._raw is not None and (now - self._last_msg_t) <= self.timeout)

        if valid:
            dt = 0.0 if self._last_t is None else clamp(now - self._last_t, 1e-3, 0.2)
            self._last_t = now
            if dt > 0.0:
                a = self._raw.linear_acceleration
                g = self._raw.angular_velocity
                st = self._filter.update(a.x, a.y, a.z, g.x, g.y, g.z, dt)
                out.roll, out.pitch = float(st.roll), float(st.pitch)
                out.yaw, out.yaw_rate = float(st.yaw), float(st.yaw_rate)
            out.valid = True
        else:
            out.valid = False
            out.roll = float(self._filter.state.roll)
            out.pitch = float(self._filter.state.pitch)
            out.yaw = float(self._filter.state.yaw)
            if self._raw is not None:
                self.get_logger().warn(
                    f'IMU 数据超时 {now - self._last_msg_t:.1f}s',
                    throttle_duration_sec=5.0)
        self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImuStateNode()
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
