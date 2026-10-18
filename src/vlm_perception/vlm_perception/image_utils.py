"""图像编解码工具：把 sensor_msgs/Image 变成 numpy，并压成送给 VLM 的 JPEG。

刻意不依赖 cv_bridge：直接按 encoding 解析 msg.data，
这样在只有 python3-opencv 的环境里也能跑，且易于单元测试。
"""
from __future__ import annotations

import base64
from typing import Optional, Tuple

import cv2
import numpy as np

# 支持的 encoding -> (dtype, channels)
_ENCODINGS = {
    'rgb8': (np.uint8, 3),
    'bgr8': (np.uint8, 3),
    'rgba8': (np.uint8, 4),
    'bgra8': (np.uint8, 4),
    'mono8': (np.uint8, 1),
    '8uc1': (np.uint8, 1),
    '8uc3': (np.uint8, 3),
    'mono16': (np.uint16, 1),
    '16uc1': (np.uint16, 1),
    '32fc1': (np.float32, 1),
}


class ImageDecodeError(ValueError):
    """无法解析的图像编码。"""


def image_msg_to_array(msg) -> np.ndarray:
    """sensor_msgs/Image -> numpy 数组（保留原始通道顺序，处理 step 对齐）。"""
    enc = (msg.encoding or '').lower()
    if enc not in _ENCODINGS:
        raise ImageDecodeError(f'unsupported encoding: {msg.encoding!r}')
    dtype, channels = _ENCODINGS[enc]
    itemsize = np.dtype(dtype).itemsize
    row_bytes = int(msg.width) * channels * itemsize
    step = int(msg.step) or row_bytes

    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if raw.size < step * int(msg.height):
        raise ImageDecodeError(
            f'truncated image: {raw.size} < {step * int(msg.height)}')
    rows = raw.reshape(int(msg.height), step)[:, :row_bytes].copy()
    if dtype is not np.uint8:
        rows = rows.view(dtype)          # 先按元素宽度重新解释，再按 (h,w,c) 切
    arr = rows.reshape(int(msg.height), int(msg.width), channels)
    if channels == 1:
        arr = arr[:, :, 0]
    return arr


def image_msg_to_bgr(msg) -> np.ndarray:
    """转成 VLM/OpenCV 需要的 BGR uint8。"""
    arr = image_msg_to_array(msg)
    enc = (msg.encoding or '').lower()
    if arr.ndim == 2:
        return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
    if arr.shape[2] == 4:
        return cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)
    if enc in ('rgb8',):
        return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
    return arr


def depth_msg_to_meters(msg) -> np.ndarray:
    """深度图 -> float32 米。支持 32FC1 / 16UC1(毫米) / mono16(毫米)。"""
    arr = image_msg_to_array(msg)
    enc = (msg.encoding or '').lower()
    if enc in ('16uc1', 'mono16'):
        return arr.astype(np.float32) / 1000.0
    return arr.astype(np.float32)


def encode_jpeg_b64(bgr: np.ndarray, max_side: int = 512,
                    quality: int = 72) -> Tuple[str, Tuple[int, int]]:
    """缩放并 JPEG 编码为 base64 字符串，返回 (b64, (h, w))。"""
    h, w = bgr.shape[:2]
    scale = min(1.0, float(max_side) / float(max(h, w)))
    if scale < 1.0:
        bgr = cv2.resize(bgr, (max(2, int(w * scale)), max(2, int(h * scale))),
                         interpolation=cv2.INTER_AREA)
    ok, buf = cv2.imencode('.jpg', bgr, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    if not ok:
        raise ImageDecodeError('cv2.imencode failed')
    return base64.b64encode(buf.tobytes()).decode('ascii'), bgr.shape[:2]


def decode_b64_image(b64: str) -> Optional[np.ndarray]:
    """base64 -> BGR；失败返回 None。Mock 服务与离线测试用。"""
    try:
        buf = np.frombuffer(base64.b64decode(b64), dtype=np.uint8)
        return cv2.imdecode(buf, cv2.IMREAD_COLOR)
    except Exception:  # noqa: BLE001 - 外部输入，任何异常都视为解码失败
        return None


def bgr_to_image_msg(bgr: np.ndarray, stamp=None, frame_id: str = 'camera_link'):
    """numpy BGR -> sensor_msgs/Image（不依赖 cv_bridge）。"""
    from sensor_msgs.msg import Image  # 延迟导入，保证本模块可离线使用

    msg = Image()
    if stamp is not None:
        msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height, msg.width = bgr.shape[:2]
    msg.encoding = 'bgr8'
    msg.is_bigendian = 0
    msg.step = int(bgr.shape[1] * bgr.shape[2])
    msg.data = np.ascontiguousarray(bgr, dtype=np.uint8).tobytes()
    return msg


def draw_overlay(bgr: np.ndarray, result: dict, latency_ms: float) -> np.ndarray:
    """把 VLM 结果画到画面上，方便 rqt_image_view 直观看决策依据。"""
    img = bgr.copy()
    h, w = img.shape[:2]
    obstacle = bool(result.get('obstacle'))
    color = (0, 0, 255) if obstacle else (0, 200, 0)
    label = str(result.get('label', ''))
    direction = str(result.get('direction', 'none'))
    action = str(result.get('action', 'go_straight'))
    risk = float(result.get('risk', 0.0))

    cv2.rectangle(img, (0, 0), (w - 1, 58), (20, 20, 20), -1)
    cv2.putText(img, f"{'OBSTACLE' if obstacle else 'CLEAR'}  risk={risk:.2f}  "
                     f"dir={direction}  action={action}",
                (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.48, color, 1, cv2.LINE_AA)
    cv2.putText(img, f'label: {label[:48]}',
                (6, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (230, 230, 230), 1, cv2.LINE_AA)
    cv2.putText(img, f'{latency_ms:.0f} ms', (w - 90, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.44, (180, 180, 180), 1, cv2.LINE_AA)

    # 三扇区分隔线，提示模型判断方位的区域
    for x in (int(w * 0.33), int(w * 0.66)):
        cv2.line(img, (x, 58), (x, h - 1), (90, 90, 90), 1)
    return img
