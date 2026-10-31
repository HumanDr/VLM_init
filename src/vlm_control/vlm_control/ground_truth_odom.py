"""真值里程计 → `/odom` + TF（仿真专用）。

为什么需要它
------------
Gazebo 的差速插件默认发布**轮式里程计**：靠轮子转速积分。小车撞墙、打滑、被
推一下，积分就会漂，越走越偏——于是 RViz 里的小车和地图对不上，A* 的路径看起来
"从墙里穿过去"。

`gazebo_ros_p3d` 能给出模型在世界里的**真值位姿**，但它：
  * 只发 Odometry，不发 TF；
  * 位姿是世界坐标，而本项目的 `/odom` 约定"出生点 = (0,0)"。

所以这个节点做两件事：
  1. 把真值位姿换算成"相对出生点"的 odom 位姿（平移 + 出生点朝向）；
  2. 发布 `/odom` 和 `odom -> base_footprint` 的 TF。

顺带把轮式里程计（`/wheel_odom`）和真值对比，定期打印漂移量——
真机上这个差值就是需要靠 SLAM/AMCL 修正的东西。
"""
from __future__ import annotations

import math
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def wrap_angle(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quat(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def to_local(x: float, y: float, yaw: float,
             origin_x: float, origin_y: float, origin_yaw: float) -> Tuple[float, float, float]:
    """世界位姿 -> 相对出生点的位姿（先平移，再按 -origin_yaw 旋转）。"""
    dx, dy = x - origin_x, y - origin_y
    c, s = math.cos(-origin_yaw), math.sin(-origin_yaw)
    return (dx * c - dy * s, dx * s + dy * c, wrap_angle(yaw - origin_yaw))


class GroundTruthOdom(Node):
    def __init__(self) -> None:
        super().__init__('ground_truth_odom')
        p = self.declare_parameter
        self.gt_topic = p('ground_truth_topic', '/ground_truth/odom').value
        self.odom_topic = p('odom_topic', '/odom').value
        self.wheel_topic = p('wheel_odom_topic', '/wheel_odom').value
        self.origin_x = float(p('origin_x', 1.5).value)
        self.origin_y = float(p('origin_y', 1.5).value)
        self.origin_yaw = float(p('origin_yaw', 0.0).value)
        self.publish_tf = bool(p('publish_tf', True).value)
        self.drift_log_period = float(p('drift_log_period', 5.0).value)

        self._wheel: Optional[Tuple[float, float]] = None
        self._last_log = 0.0
        self._max_drift = 0.0

        self._pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self._tf = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(Odometry, self.gt_topic, self._on_gt, 10)
        self.create_subscription(Odometry, self.wheel_topic, self._on_wheel, 10)
        self.get_logger().info(
            f'真值里程计启动: {self.gt_topic} -> {self.odom_topic} '
            f'(origin=({self.origin_x:.2f},{self.origin_y:.2f},{self.origin_yaw:.2f}), '
            f'publish_tf={self.publish_tf})')

    def _on_wheel(self, msg: Odometry) -> None:
        self._wheel = (float(msg.pose.pose.position.x),
                       float(msg.pose.pose.position.y))

    def _on_gt(self, msg: Odometry) -> None:
        pos = msg.pose.pose.position
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        lx, ly, lyaw = to_local(float(pos.x), float(pos.y), yaw,
                                self.origin_x, self.origin_y, self.origin_yaw)

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'odom'
        out.child_frame_id = 'base_footprint'
        out.pose.pose.position.x = lx
        out.pose.pose.position.y = ly
        out.pose.pose.orientation.z = math.sin(lyaw / 2.0)
        out.pose.pose.orientation.w = math.cos(lyaw / 2.0)
        out.twist = msg.twist
        self._pub.publish(out)

        if self._tf is not None:
            t = TransformStamped()
            t.header = out.header
            t.child_frame_id = out.child_frame_id
            t.transform.translation.x = lx
            t.transform.translation.y = ly
            t.transform.rotation = out.pose.pose.orientation
            self._tf.sendTransform(t)

        # 轮式里程计漂移诊断
        if self._wheel is not None:
            drift = math.hypot(self._wheel[0] - lx, self._wheel[1] - ly)
            self._max_drift = max(self._max_drift, drift)
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self._last_log > self.drift_log_period:
                self._last_log = now
                if drift > 0.10:
                    self.get_logger().warn(
                        f'轮式里程计漂移 {drift:.2f} m（最大 {self._max_drift:.2f} m）'
                        '——已用真值位姿控制，地图仍对齐')
                else:
                    self.get_logger().info(
                        f'轮式里程计漂移 {drift:.2f} m（最大 {self._max_drift:.2f} m）')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GroundTruthOdom()
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
