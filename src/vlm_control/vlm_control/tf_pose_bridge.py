"""TF -> 位姿桥：把任意 SLAM 框架的 TF 变成项目约定的 `/slam/odom`。

RTAB-Map（以及 cartographer、slam_toolbox）都以 TF 的形式给出修正后的位姿，
而本项目的控制器/A* 只订阅一个 `nav_msgs/Odometry` 话题。这个节点负责桥接：

    TF map -> base_footprint  ==>  /slam/odom (nav_msgs/Odometry, frame=map)

这样换 SLAM 框架时，下游（控制器、A*、rviz）一行都不用改。
"""
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


class TfPoseBridge(Node):
    def __init__(self) -> None:
        super().__init__('tf_pose_bridge')
        p = self.declare_parameter
        self.target_frame = p('target_frame', 'map').value
        self.source_frame = p('source_frame', 'base_footprint').value
        self.odom_topic = p('odom_topic', '/slam/odom').value
        self.rate_hz = float(p('rate_hz', 20.0).value)
        self.timeout = float(p('timeout', 0.15).value)

        self._buf = Buffer()
        self._listener = TransformListener(self._buf, self)
        self._pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self._warned = False
        self.create_timer(1.0 / max(1.0, self.rate_hz), self._tick)
        self.get_logger().info(
            f'TF 位姿桥启动: {self.target_frame} -> {self.source_frame} '
            f'=> {self.odom_topic} @ {self.rate_hz:.0f}Hz')

    def _tick(self) -> None:
        try:
            t = self._buf.lookup_transform(self.target_frame, self.source_frame,
                                           rclpy.time.Time(),
                                           timeout=rclpy.duration.Duration(
                                               seconds=self.timeout))
        except TransformException as exc:
            if not self._warned:
                self._warned = True
                self.get_logger().warn(
                    f'等 TF {self.target_frame}->{self.source_frame}: {exc}')
            return
        if self._warned:
            self._warned = False
            self.get_logger().info('TF 已就绪，开始发布位姿')

        tr = t.transform.translation
        q = t.transform.rotation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        m = Odometry()
        m.header.stamp = t.header.stamp
        m.header.frame_id = self.target_frame
        m.child_frame_id = self.source_frame
        m.pose.pose.position.x = float(tr.x)
        m.pose.pose.position.y = float(tr.y)
        m.pose.pose.position.z = float(tr.z)
        m.pose.pose.orientation = q
        self._pub.publish(m)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TfPoseBridge()
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
