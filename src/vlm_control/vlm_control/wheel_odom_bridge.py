"""轮式里程计桥：把 `/wheel_odom` 变成 `/odom` + TF `odom -> base_footprint`。

为什么需要它
------------
差速插件只发 `/wheel_odom`，且不再发 TF（见 URDF）。而 RTAB-Map / rviz / 控制器
都需要一条稳定的 `odom -> base_footprint`。

之前用的是激光 ICP 里程计（`icp_odometry`），但它在 2D 仿真里容易失锁：
`Registration failed: limit out of bounds ... ratio=0.000000`，一旦失锁，
TF 就冻结在那一刻（rviz 里机器人变白、不动），rtabmap 也跟着停。

轮式里程计虽然会漂，但**永不失锁**；漂移交给 RTAB-Map 的回环 + 位姿图去修，
这正是 TurtleBot 那套经典配置（wheel odom + RGB-D/LiDAR SLAM）。
"""
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class WheelOdomBridge(Node):
    def __init__(self) -> None:
        super().__init__('wheel_odom_bridge')
        p = self.declare_parameter
        self.wheel_topic = p('wheel_odom_topic', '/wheel_odom').value
        self.odom_topic = p('odom_topic', '/odom').value
        self.publish_tf = bool(p('publish_tf', True).value)
        self.frame_id = p('frame_id', 'odom').value
        self.base_frame = p('base_frame', 'base_footprint').value

        self._pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self._tf = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(Odometry, self.wheel_topic, self._on_wheel, 10)
        self._n = 0
        self.get_logger().info(
            f'轮式里程计桥: {self.wheel_topic} -> {self.odom_topic} '
            f'+ TF {self.frame_id}->{self.base_frame}')

    def _on_wheel(self, msg: Odometry) -> None:
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame_id
        out.child_frame_id = self.base_frame
        out.pose = msg.pose
        out.twist = msg.twist
        self._pub.publish(out)

        if self._tf is not None:
            q = msg.pose.pose.orientation
            t = TransformStamped()
            t.header = out.header
            t.child_frame_id = self.base_frame
            t.transform.translation.x = float(msg.pose.pose.position.x)
            t.transform.translation.y = float(msg.pose.pose.position.y)
            t.transform.translation.z = float(msg.pose.pose.position.z)
            t.transform.rotation = q
            self._tf.sendTransform(t)

        self._n += 1
        if self._n == 5:
            self.get_logger().info('已收到轮式里程计，TF 开始发布')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WheelOdomBridge()
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
