"""IMU 姿态估计：互补滤波（陀螺积分 + 加速度重力方向校正）。

没有磁力计，yaw 会缓慢漂移，所以它只用于"短时间航向保持"与"倾覆急停"，
不用于定位。真机上想要更稳的 yaw，后续可换 EKF（robot_localization）或加磁力计。
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from .planner import clamp, wrap_angle


@dataclass
class ImuState:
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    yaw_rate: float = 0.0
    initialized: bool = False


class ComplementaryFilter:
    def __init__(self, alpha: float = 0.98, snap_threshold: float = 0.5) -> None:
        self.alpha = clamp(alpha, 0.0, 1.0)
        # 陀螺权重高（抗噪、抗线加速度干扰），但姿态突变时必须快速收敛，
        # 否则"跌倒/被抬起"要 1~2 秒才触发急停，这在真机上是致命的。
        self.snap_threshold = float(snap_threshold)
        self.state = ImuState()
        self._gyro_bias_z = 0.0
        self._bias_samples = 0

    def update(self, ax: float, ay: float, az: float,
               gx: float, gy: float, gz: float, dt: float) -> ImuState:
        dt = clamp(dt, 1e-4, 0.5)
        s = self.state

        # 加速度计给出的姿态（仅在接近静止时可信）
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        acc_roll = math.atan2(ay, az) if abs(az) > 1e-6 or abs(ay) > 1e-6 else 0.0
        acc_pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))

        if not s.initialized:
            s.roll, s.pitch = acc_roll, acc_pitch
            s.yaw = 0.0
            s.initialized = True
            self._gyro_bias_z = gz
        else:
            a = self.alpha
            pred_roll = s.roll + gx * dt
            pred_pitch = s.pitch + gy * dt
            s.roll = a * pred_roll + (1.0 - a) * acc_roll
            s.pitch = a * pred_pitch + (1.0 - a) * acc_pitch
            # 与加速度计严重不一致 = 姿态真的变了（跌倒/侧翻），半数拉到测量值
            if abs(acc_roll - pred_roll) > self.snap_threshold:
                s.roll = 0.5 * (s.roll + acc_roll)
            if abs(acc_pitch - pred_pitch) > self.snap_threshold:
                s.pitch = 0.5 * (s.pitch + acc_pitch)

            # 静止时估计零偏（|g| 接近 9.81 且角速度小）
            if abs(norm - 9.81) < 0.35 and abs(gz) < 0.08:
                self._bias_samples += 1
                beta = 1.0 / min(200, self._bias_samples + 1)
                self._gyro_bias_z += beta * (gz - self._gyro_bias_z)
            s.yaw_rate = gz - self._gyro_bias_z
            s.yaw = wrap_angle(s.yaw + s.yaw_rate * dt)

        return s
