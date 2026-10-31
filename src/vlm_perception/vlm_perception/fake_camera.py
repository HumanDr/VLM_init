"""假相机：不启动 Gazebo 也能把整条链路跑起来的合成相机 + 深度 + IMU。

场景由话题 /test/scene 切换（std_msgs/String）：
    free | center | left | right | crowded | closed | tilt

'tilt' 会让 IMU 输出 1.2 rad 的横滚，用来验证倾覆急停。
"""
from __future__ import annotations

import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, LaserScan
from std_msgs.msg import String

from .image_utils import bgr_to_image_msg

RED = (40, 40, 205)
BLUE = (200, 110, 40)
YELLOW = (40, 210, 220)
GREY = (95, 95, 100)


def _euler_to_quat(roll: float, pitch: float, yaw: float):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


class FakeCamera(Node):
    def __init__(self) -> None:
        super().__init__('fake_camera')
        p = self.declare_parameter
        self.width = int(p('width', 640).value)
        self.height = int(p('height', 480).value)
        self.rate = float(p('rate_hz', 10.0).value)
        self.scene = str(p('scene', 'free').value)
        self.frame_id = str(p('frame_id', 'camera_link').value)
        self.publish_depth = bool(p('publish_depth', True).value)
        self.publish_imu = bool(p('publish_imu', True).value)
        self.publish_scan = bool(p('publish_scan', True).value)
        self.imu_rate = float(p('imu_rate_hz', 50.0).value)
        self.max_range = float(p('max_range', 6.0).value)

        self._pub_img = self.create_publisher(Image, '/camera/image_raw', qos_profile_sensor_data)
        self._pub_depth = (self.create_publisher(Image, '/camera/depth/image_raw',
                                                 qos_profile_sensor_data)
                           if self.publish_depth else None)
        self._pub_imu = (self.create_publisher(Imu, '/imu/data', qos_profile_sensor_data)
                         if self.publish_imu else None)
        self._pub_scan = (self.create_publisher(LaserScan, '/scan',
                                                qos_profile_sensor_data)
                          if self.publish_scan else None)
        self.create_subscription(String, '/test/scene', self._on_scene, 10)

        self.create_timer(1.0 / max(0.5, self.rate), self._tick_image)
        if self._pub_imu is not None:
            self.create_timer(1.0 / max(1.0, self.imu_rate), self._tick_imu)
        self.get_logger().info(f'假相机启动 scene={self.scene} '
                               f'{self.width}x{self.height}@{self.rate}Hz')

    def _on_scene(self, msg: String) -> None:
        self.scene = msg.data.strip().lower()
        self.get_logger().info(f'切换场景 -> {self.scene}')

    # ------------------------------------------------------------ scene
    def _render(self):
        w, h = self.width, self.height
        img = np.zeros((h, w, 3), np.uint8)
        # 天花板/远处渐变
        for y in range(h):
            v = int(52 + 38 * (y / h))
            img[y, :] = (v, v, v + 4)
        # 地面
        cv2.rectangle(img, (0, int(h * 0.52)), (w, h), (88, 90, 94), -1)
        # 走廊侧墙
        cv2.fillConvexPoly(img, np.array(
            [[0, h], [0, int(h * 0.12)], [int(w * 0.20), int(h * 0.34)],
             [int(w * 0.20), int(h * 0.86)]], np.int32), (72, 74, 78))
        cv2.fillConvexPoly(img, np.array(
            [[w, h], [w, int(h * 0.12)], [int(w * 0.80), int(h * 0.34)],
             [int(w * 0.80), int(h * 0.86)]], np.int32), (78, 80, 84))
        # 地面透视线
        for x in range(-2, 3):
            x0 = int(w * 0.5 + x * w * 0.16)
            cv2.line(img, (int(w * 0.5), int(h * 0.54)), (x0, h), (70, 72, 76), 2)

        depth = np.full((self.height // 2, self.width // 2), self.max_range, np.float32)
        dw, dh = depth.shape[1], depth.shape[0]

        self._draw_walls_depth(depth, dw, dh)

        scene = self.scene
        specs = []
        if scene == 'center':
            specs = [(0.50, 0.30, RED, 0.30, 'box')]
        elif scene == 'left':
            specs = [(0.22, 0.24, RED, 0.35, 'box')]
        elif scene == 'right':
            specs = [(0.78, 0.24, YELLOW, 0.30, 'box')]
        elif scene == 'closed':
            specs = [(0.50, 0.62, RED, 0.18, 'box')]
        elif scene == 'crowded':
            specs = [(0.24, 0.26, RED, 0.35, 'box'),
                     (0.76, 0.24, BLUE, 0.40, 'can')]
        elif scene == 'person':
            specs = [(0.50, 0.22, BLUE, 0.30, 'person')]
        for cx, size, color, dist, kind in specs:
            self._draw_obstacle(img, depth, cx, size, color, kind)
            self._stamp_depth(depth, cx, size, dist)
        return img, depth

    def _draw_walls_depth(self, depth: np.ndarray, dw: int, dh: int) -> None:
        depth[:, : int(dw * 0.10)] = 2.2
        depth[:, int(dw * 0.90):] = 2.2
        depth[int(dh * 0.15):, : int(dw * 0.18)] = 2.6
        depth[int(dh * 0.15):, int(dw * 0.82):] = 2.6

    def _draw_obstacle(self, img, depth, cx_frac, size_frac, color, kind) -> None:
        h, w = img.shape[:2]
        cx = int(w * cx_frac)
        bw = int(w * size_frac)
        bh = int(bw * 0.95)
        base_y = int(h * 0.90)
        x0, x1 = cx - bw // 2, cx + bw // 2
        y1 = base_y
        y0 = y1 - bh
        if kind == 'person':
            body_w = max(8, int(bw * 0.55))
            cv2.rectangle(img, (cx - body_w // 2, y0 + int(bh * 0.18)),
                          (cx + body_w // 2, y1), color, -1)
            cv2.circle(img, (cx, y0 + int(bh * 0.14)), max(6, int(bw * 0.16)),
                       (150, 170, 195), -1)
            return
        if kind == 'can':
            cv2.ellipse(img, (cx, y1 - 4), (bw // 2, max(4, bw // 12)), 0, 0, 360,
                        color, -1)
            cv2.rectangle(img, (x0, y0 + int(bh * 0.06)), (x1, y1 - 4), color, -1)
            cv2.ellipse(img, (cx, y0 + int(bh * 0.06)), (bw // 2, max(4, bw // 12)),
                        0, 0, 360, tuple(min(255, c + 30) for c in color), -1)
            return
        # 立方体：正面 + 顶面
        cv2.rectangle(img, (x0, y0), (x1, y1), color, -1)
        top = np.array([[x0, y0], [x1, y0],
                        [x1 - int(bw * 0.16), y0 - int(bh * 0.20)],
                        [x0 - int(bw * 0.16), y0 - int(bh * 0.20)]], np.int32)
        lighter = tuple(min(255, c + 35) for c in color)
        cv2.fillConvexPoly(img, top, lighter)
        cv2.rectangle(img, (x0, y0), (x1, y1), (30, 30, 35), 2)
        # 影子
        cv2.ellipse(img, (cx, y1 + 4), (bw // 2, max(4, bw // 10)), 0, 0, 360,
                    (70, 72, 76), -1)

    def _stamp_depth(self, depth, cx_frac, size_frac, dist) -> None:
        dh, dw = depth.shape
        c0 = int(dw * max(0.0, cx_frac - size_frac / 2))
        c1 = int(dw * min(1.0, cx_frac + size_frac / 2))
        # 只标记"地平线以上"的行：深度节点会按相机外参剔除地面点，
        # 画在最下方（几何上低于地面）的像素会被正确过滤掉。
        r0, r1 = int(dh * 0.30), int(dh * 0.52)
        depth[r0:r1, max(0, c0):max(1, c1)] = dist

    # ------------------------------------------------------------ publish
    def _laser_arc(self):
        """按场景返回 (中心方位角, 距离)；与深度图里的障碍保持一致。"""
        scene = self.scene
        arcs = {
            'center': (0.0, 0.30),
            'left': (0.55, 0.35),
            'right': (-0.55, 0.30),
            'closed': (0.0, 0.18),
            'crowded': (0.0, 0.35),
            'person': (0.0, 0.30),
        }
        return arcs.get(scene)

    def _publish_laser(self, stamp) -> None:
        n = 720
        msg = LaserScan()
        msg.header.stamp = stamp
        msg.header.frame_id = 'laser_link'
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = 2.0 * math.pi / (n - 1)
        msg.range_min = 0.12
        msg.range_max = 12.0
        ranges = [7.0] * n
        arc = self._laser_arc()
        if arc is not None:
            bearing, dist = arc
            half = 0.25          # 障碍在雷达上占 ±14°
            for i in range(n):
                a = msg.angle_min + i * msg.angle_increment
                if abs(a - bearing) <= half:
                    ranges[i] = dist
        msg.ranges = ranges
        self._pub_scan.publish(msg)

    def _tick_image(self) -> None:
        img, depth = self._render()
        stamp = self.get_clock().now().to_msg()
        self._pub_img.publish(bgr_to_image_msg(img, stamp, self.frame_id))
        if self._pub_depth is not None:
            dmsg = Image()
            dmsg.header.stamp = stamp
            dmsg.header.frame_id = 'camera_depth_optical_frame'
            dmsg.height, dmsg.width = depth.shape
            dmsg.encoding = '32FC1'
            dmsg.is_bigendian = 0
            dmsg.step = depth.shape[1] * 4
            dmsg.data = np.ascontiguousarray(depth, dtype=np.float32).tobytes()
            self._pub_depth.publish(dmsg)
        if self._pub_scan is not None:
            self._publish_laser(stamp)

    def _tick_imu(self) -> None:
        roll = 1.2 if self.scene == 'tilt' else 0.0
        pitch = -0.05
        yaw = 0.0
        q = _euler_to_quat(roll, pitch, yaw)
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        msg.orientation.x, msg.orientation.y = q[0], q[1]
        msg.orientation.z, msg.orientation.w = q[2], q[3]
        msg.linear_acceleration.x = 9.81 * math.sin(pitch)
        msg.linear_acceleration.z = 9.81 * math.cos(pitch) * math.cos(roll)
        msg.linear_acceleration.y = -9.81 * math.sin(roll)
        msg.angular_velocity.z = 0.02
        self._pub_imu.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeCamera()
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
