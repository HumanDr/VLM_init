"""RGB-D 视觉里程计：ORB 特征 + PnP，输出平面相对运动。

在 SLAM 里的角色：给前端提供一路**独立的**运动估计，和激光雷达扫描匹配、
IMU 陀螺、轮式里程计一起约束位姿。

为什么需要它：
  * 雷达在长走廊/空旷处几何退化（能看到的面太少）；
  * 轮式里程计会打滑；
  * 相机在纹理丰富处更准。
三者互补，任一路失效就退化成另外两路，不会整体挂掉。

做法：ORB 特征 + 前一帧深度图反投影出 3D 点，和当前帧 2D 点做 solvePnPRansac，
再取平面分量（tvec 的 x、y 和旋转矩阵的 yaw）。特征/内点不够就返回 None。
"""
from __future__ import annotations

import math
from typing import Dict, Optional, Tuple

import numpy as np

Pose = Tuple[float, float, float]     # (dx, dy, dyaw)


class VisualOdometry:
    def __init__(self, n_features: int = 400, min_inliers: int = 15,
                 max_depth: float = 8.0) -> None:
        self.n_features = int(n_features)
        self.min_inliers = int(min_inliers)
        self.max_depth = float(max_depth)
        self._orb = None
        self._matcher = None
        self._prev_des = None
        self._prev_pts3d: Dict[int, Tuple[float, float, float]] = {}
        self._K: Optional[np.ndarray] = None
        self.last_info = '未初始化'

    def _ensure_cv(self) -> bool:
        if self._orb is not None:
            return True
        try:
            import cv2
        except ImportError:
            self.last_info = 'opencv 不可用'
            return False
        self._orb = cv2.ORB_create(nfeatures=self.n_features)
        self._matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        return True

    # ------------------------------------------------------------ 主接口
    def update(self, rgb: np.ndarray, depth: np.ndarray,
               K: np.ndarray) -> Optional[Pose]:
        """返回相机坐标系下的 (dx, dy, dyaw)；失败返回 None。"""
        if not self._ensure_cv() or rgb is None or depth is None:
            return None
        import cv2

        self._K = np.asarray(K, dtype=np.float64)
        gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY) if rgb.ndim == 3 else rgb
        if depth.shape[:2] != gray.shape[:2]:
            depth = cv2.resize(depth, (gray.shape[1], gray.shape[0]),
                               interpolation=cv2.INTER_NEAREST)
        depth = np.asarray(depth, dtype=np.float32)
        if depth.size and float(depth.max()) <= 1.5:
            depth = depth * 10.0          # 概率深度图 (0..1) 还原成米

        kp, des = self._orb.detectAndCompute(gray, None)
        n_kp = 0 if des is None else len(kp)
        if des is None or n_kp < self.min_inliers:
            self.last_info = f'特征太少({n_kp})'
            self._store(depth, kp, des)
            return None
        if self._prev_des is None or not self._prev_pts3d:
            self._store(depth, kp, des)
            self.last_info = f'首帧({n_kp})'
            return None

        try:
            matches = self._matcher.match(self._prev_des, des)
        except cv2.error:
            self._store(depth, kp, des)
            self.last_info = '匹配失败'
            return None
        if len(matches) < self.min_inliers:
            self._store(depth, kp, des)
            self.last_info = f'匹配太少({len(matches)})'
            return None

        obj, img = [], []
        for m in matches:
            p3 = self._prev_pts3d.get(m.queryIdx)
            if p3 is None:
                continue
            obj.append(p3)
            img.append(kp[m.trainIdx].pt)
        if len(obj) < self.min_inliers:
            self._store(depth, kp, des)
            self.last_info = f'3D 点太少({len(obj)})'
            return None

        obj = np.asarray(obj, dtype=np.float64)
        img = np.asarray(img, dtype=np.float64)
        try:
            ok, rvec, tvec, inliers = cv2.solvePnPRansac(
                obj, img, self._K, np.zeros((4, 1)),
                flags=cv2.SOLVEPNP_ITERATIVE,
                reprojectionError=3.0, iterationsCount=80, confidence=0.99)
        except cv2.error:
            ok = False
        n_in = 0 if not ok or inliers is None else int(len(inliers))
        self._store(depth, kp, des)
        if not ok or n_in < self.min_inliers:
            self.last_info = f'PnP 失败(内点 {n_in})'
            return None

        R, _ = cv2.Rodrigues(rvec)
        dyaw = math.atan2(R[1, 0], R[0, 0])
        dx, dy = float(tvec[0, 0]), float(tvec[1, 0])
        if not (-1.5 < dx < 1.5 and -1.5 < dy < 1.5 and abs(dyaw) < 1.0):
            self.last_info = f'位移异常({dx:.2f},{dy:.2f},{dyaw:.2f})'
            return None
        self.last_info = f'内点 {n_in}'
        return (dx, dy, dyaw)

    # ------------------------------------------------------------ 内部
    def _store(self, depth, kp, des) -> None:
        """把当前帧的特征点反投影成 3D（相机系），留给下一帧做 PnP。"""
        self._prev_des = des
        pts: Dict[int, Tuple[float, float, float]] = {}
        if des is None or self._K is None:
            self._prev_pts3d = {}
            return
        fx, fy = self._K[0, 0], self._K[1, 1]
        cx, cy = self._K[0, 2], self._K[1, 2]
        h, w = depth.shape[:2]
        for i, k in enumerate(kp):
            u, v = int(round(k.pt[0])), int(round(k.pt[1]))
            if not (0 <= v < h and 0 <= u < w):
                continue
            d = float(depth[v, u])
            if d <= 0.05 or d > self.max_depth:
                continue
            pts[i] = ((u - cx) * d / fx, (v - cy) * d / fy, d)
        self._prev_pts3d = pts
