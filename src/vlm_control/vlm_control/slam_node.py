"""SLAM 节点：把 slam_core（雷达+IMU+轮速+RGB-D VO）接成 ROS 节点。

发布：
    /slam/odom   nav_msgs/Odometry   地图系（本项目的 odom）下的位姿
    /map         nav_msgs/OccupancyGrid  实时建图（供 A* 规划）
    TF           odom(map) -> base_footprint

为什么不用现成的框架（RTAB-Map / slam_toolbox / ORB-SLAM3）
-----------------------------------------------------------
它们确实更强，但都需要 `apt install`（要 sudo）。这个节点是**零依赖**版本：
同样的三传感器输入、扫描匹配 + 位姿图回环 + 占用栅格，精度差一些，但开箱即用、
可离线仿真验证。装了 RTAB-Map 的话把 launch 参数切成 `localization:=rtabmap` 即可。
"""
from __future__ import annotations

import math
from typing import Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan
from tf2_ros import TransformBroadcaster

from .slam_core import SlamCore, wrap


def _yaw(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class SlamNode(Node):
    def __init__(self) -> None:
        super().__init__('slam_node')
        p = self.declare_parameter
        self.scan_topic = p('scan_topic', '/scan').value
        self.imu_topic = p('imu_topic', '/imu/data').value
        self.wheel_topic = p('wheel_odom_topic', '/wheel_odom').value
        self.odom_topic = p('odom_topic', '/slam/odom').value
        self.map_topic = p('map_topic', '/map').value
        self.map_frame = p('map_frame', 'odom').value
        self.base_frame = p('base_frame', 'base_footprint').value
        self.max_range = float(p('max_range', 12.0).value)
        self.map_publish_period = float(p('map_publish_period', 1.0).value)
        self.enable_vo = bool(p('enable_vo', True).value)
        self.vo_rate = float(p('vo_rate_hz', 5.0).value)
        self.rgb_topic = p('rgb_topic', '/camera_depth/image_raw').value
        self.depth_topic = p('depth_topic', '/camera_depth/depth/image_raw').value
        self.info_topic = p('camera_info_topic', '/camera_depth/camera_info').value
        self.keyframe_dist = float(p('keyframe_dist', 0.35).value)
        self.keyframe_angle = float(p('keyframe_angle', 0.20).value)
        self.loop_radius = float(p('loop_radius', 3.0).value)

        self.slam = SlamCore(max_range=self.max_range,
                             keyframe_dist=self.keyframe_dist,
                             keyframe_angle=self.keyframe_angle,
                             loop_radius=self.loop_radius)
        self._pub_odom = self.create_publisher(Odometry, self.odom_topic, 10)
        self._pub_map = self.create_publisher(OccupancyGrid, self.map_topic, 1)
        self._tf = TransformBroadcaster(self)

        self._wheel_pose: Optional[Tuple[float, float]] = None
        self._wheel_prev: Optional[Tuple[float, float]] = None
        self._yaw_acc = 0.0
        self._last_imu: Optional[float] = None
        self._last_map_t = 0.0
        self._last_vo_t = 0.0
        self._vo_delta = None
        self._K = None

        self.create_subscription(LaserScan, self.scan_topic, self._on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(Imu, self.imu_topic, self._on_imu,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, self.wheel_topic, self._on_wheel, 10)
        if self.enable_vo:
            from .visual_odometry import VisualOdometry
            self._vo = VisualOdometry()
            self.create_subscription(Image, self.rgb_topic, self._on_rgb,
                                     qos_profile_sensor_data)
            self.create_subscription(Image, self.depth_topic, self._on_depth,
                                     qos_profile_sensor_data)
            self.create_subscription(CameraInfo, self.info_topic, self._on_info,
                                     qos_profile_sensor_data)
            self._rgb = None
            self._depth = None
        else:
            self._vo = None

        self.get_logger().info(
            f'SLAM 启动: scan={self.scan_topic} imu={self.imu_topic} '
            f'wheel={self.wheel_topic} vo={"on" if self.enable_vo else "off"} '
            f'-> {self.odom_topic} / {self.map_topic}')

    # ------------------------------------------------------------ 回调
    def _on_wheel(self, msg: Odometry) -> None:
        self._wheel_pose = (float(msg.pose.pose.position.x),
                            float(msg.pose.pose.position.y))

    def _on_imu(self, msg: Imu) -> None:
        t = self.get_clock().now().nanoseconds / 1e9
        if self._last_imu is not None:
            dt = max(0.0, min(0.5, t - self._last_imu))
            self._yaw_acc += msg.angular_velocity.z * dt
        self._last_imu = t

    def _on_info(self, msg: CameraInfo) -> None:
        k = msg.k
        self._K = np.array([[k[0], 0.0, k[2]], [0.0, k[4], k[5]], [0.0, 0.0, 1.0]])

    def _on_rgb(self, msg: Image) -> None:
        self._rgb = self._to_np(msg)

    def _on_depth(self, msg: Image) -> None:
        self._depth = self._to_np(msg)
        self._maybe_vo()

    @staticmethod
    def _to_np(msg: Image):
        try:
            if msg.encoding in ('rgb8', 'bgr8'):
                return np.frombuffer(msg.data, np.uint8).reshape(
                    msg.height, msg.width, 3).copy()
            if msg.encoding in ('32FC1', '16UC1', 'mono16'):
                dt = np.float32 if msg.encoding == '32FC1' else np.uint16
                return np.frombuffer(msg.data, dt).reshape(
                    msg.height, msg.width).copy()
            if msg.encoding == 'mono8':
                return np.frombuffer(msg.data, np.uint8).reshape(
                    msg.height, msg.width).copy()
        except (ValueError, TypeError):
            return None
        return None

    def _maybe_vo(self) -> None:
        if self._vo is None or self._rgb is None or self._depth is None:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last_vo_t < 1.0 / max(0.5, self.vo_rate):
            return
        self._last_vo_t = now
        K = self._K if self._K is not None else np.array(
            [[320.0, 0, 160.0], [0, 320.0, 120.0], [0, 0, 1]])
        d = self._vo.update(self._rgb, self._depth, K)
        if d is not None:
            self._vo_delta = d
            if self._vo.last_info.startswith('内点'):
                self.get_logger().debug(f'VO {self._vo.last_info} d=({d[0]:.3f},'
                                        f'{d[1]:.3f},{d[2]:.3f})')

    # ------------------------------------------------------------ 主循环
    def _on_scan(self, msg: LaserScan) -> None:
        n = len(msg.ranges)
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        angles = msg.angle_min + np.arange(n) * msg.angle_increment

        wheel_delta = (0.0, 0.0, 0.0)
        if self._wheel_pose is not None and self._wheel_prev is not None:
            dx = self._wheel_pose[0] - self._wheel_prev[0]
            dy = self._wheel_pose[1] - self._wheel_prev[1]
            c, s = math.cos(-self.slam.pose[2]), math.sin(-self.slam.pose[2])
            wheel_delta = (c * dx - s * dy, s * dx + c * dy, 0.0)
        self._wheel_prev = self._wheel_pose

        imu_delta = (0.0, 0.0, self._yaw_acc)
        self._yaw_acc = 0.0
        vo_delta = self._vo_delta
        self._vo_delta = None

        res = self.slam.process(ranges, angles, wheel_delta=wheel_delta,
                                imu_delta=imu_delta, vo_delta=vo_delta)
        self._publish_odom(res.pose)
        self._publish_tf(res.pose)

        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last_map_t >= self.map_publish_period:
            self._last_map_t = now
            self._publish_map()
        if res.loop_closed:
            self.get_logger().info(
                f'回环闭合：位姿已重优化（关键帧 {res.keyframes}），地图重建')

    # ------------------------------------------------------------ 发布
    def _publish_odom(self, pose) -> None:
        m = Odometry()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.map_frame
        m.child_frame_id = self.base_frame
        m.pose.pose.position.x = float(pose[0])
        m.pose.pose.position.y = float(pose[1])
        m.pose.pose.orientation.z = math.sin(pose[2] / 2.0)
        m.pose.pose.orientation.w = math.cos(pose[2] / 2.0)
        self._pub_odom.publish(m)

    def _publish_tf(self, pose) -> None:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = float(pose[0])
        t.transform.translation.y = float(pose[1])
        t.transform.rotation.z = math.sin(pose[2] / 2.0)
        t.transform.rotation.w = math.cos(pose[2] / 2.0)
        self._tf.sendTransform(t)

    def _publish_map(self) -> None:
        data = self.slam.map.occupancy_int8()
        m = OccupancyGrid()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.map_frame
        m.info.resolution = self.slam.map.res
        m.info.width = self.slam.map.width
        m.info.height = self.slam.map.height
        m.info.origin.position.x = self.slam.map.origin[0]
        m.info.origin.position.y = self.slam.map.origin[1]
        m.info.origin.orientation.w = 1.0
        m.data = data.reshape(-1).tolist()
        self._pub_map.publish(m)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SlamNode()
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
