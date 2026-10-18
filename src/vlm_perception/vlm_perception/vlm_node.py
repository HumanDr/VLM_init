"""VLM 感知节点：相机图像 -> Qwen-VL -> 结构化障碍物判断。

设计要点（重点解决"大模型慢"与"避障要快"的矛盾）：
  1. 推理放在独立工作线程，图像回调只做"存最新帧"，绝不阻塞 executor；
  2. 单帧在途策略：上一帧没回来就跳过新帧，避免请求堆积把延迟越拖越大；
  3. 结果带 stale 标志与 latency，下游据此做降级（减速/看门狗）。
"""
from __future__ import annotations

import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from vlm_avoid_interfaces.msg import VlmObstacle

from .image_utils import (bgr_to_image_msg, draw_overlay, encode_jpeg_b64,
                          image_msg_to_bgr)
from .vlm_client import SYSTEM_PROMPT, USER_PROMPT, VlmClient


class VlmObstacleNode(Node):
    def __init__(self) -> None:
        super().__init__('vlm_obstacle_node')

        p = self.declare_parameter
        self.image_topic = p('image_topic', '/camera/image_raw').value
        self.result_topic = p('result_topic', '/perception/vlm/result').value
        self.annotated_topic = p('annotated_topic',
                                 '/perception/vlm/annotated').value
        self.vlm_url = p('vlm_url', 'http://127.0.0.1:11434').value
        self.api_style = p('api_style', 'ollama').value
        self.model = p('model', 'qwen2.5vl:3b').value
        self.rate_hz = float(p('rate_hz', 1.0).value)
        self.max_side = int(p('max_side', 512).value)
        self.jpeg_quality = int(p('jpeg_quality', 72).value)
        self.timeout = float(p('timeout', 10.0).value)
        self.num_predict = int(p('num_predict', 128).value)
        self.num_ctx = int(p('num_ctx', 4096).value)
        self.keep_alive = str(p('keep_alive', '30m').value)
        self.temperature = float(p('temperature', 0.0).value)
        self.risk_ema_alpha = float(p('risk_ema_alpha', 0.5).value)
        self.publish_debug_image = bool(p('publish_debug_image', True).value)
        self.system_prompt = str(p('system_prompt', SYSTEM_PROMPT).value)
        self.user_prompt = str(p('user_prompt', USER_PROMPT).value)
        self.fail_publish_risk = float(p('fail_publish_risk', 0.6).value)

        self._client = VlmClient(
            base_url=self.vlm_url, model=self.model, api_style=self.api_style,
            timeout=self.timeout, num_predict=self.num_predict, num_ctx=self.num_ctx,
            keep_alive=self.keep_alive, temperature=self.temperature,
            system_prompt=self.system_prompt, user_prompt=self.user_prompt)

        self._pub = self.create_publisher(VlmObstacle, self.result_topic, 10)
        self._pub_debug = (self.create_publisher(Image, self.annotated_topic, 2)
                           if self.publish_debug_image else None)

        self._lock = threading.Lock()
        self._frame = None          # (seq, bgr, stamp)
        self._seq = 0
        self._new_frame = threading.Event()
        self._stop = threading.Event()
        self._risk_ema = 0.0
        self._last_ok = None        # 最近一次成功结果
        self._n_ok = 0
        self._n_fail = 0
        self._timeout_hinted = False
        self._last_log = 0.0

        self._sub = self.create_subscription(
            Image, self.image_topic, self._on_image, qos_profile_sensor_data)

        self.get_logger().info(
            f'VLM 感知节点启动: image={self.image_topic} -> {self.result_topic} '
            f'backend={self.api_style} model={self.model} url={self.vlm_url} '
            f'rate={self.rate_hz:.1f}Hz')
        ok, detail = self._client.health()
        if ok:
            self.get_logger().info(f'VLM 后端探测: {detail}')
            if 'not pulled' in detail:
                self.get_logger().warn(
                    f'模型 {self.model} 似乎还没下载：ollama pull {self.model}')
        else:
            self.get_logger().warn(
                f'VLM 后端不可达（{detail}）。管线仍会运行，但输出将标记 stale，'
                '控制器会降级为低速/停车。检查 Ollama 是否启动。')

        self.add_on_set_parameters_callback(self._on_set_params)
        self._worker = threading.Thread(target=self._loop, name='vlm_worker', daemon=True)
        self._worker.start()

    # ------------------------------------------------------------ params
    def _on_set_params(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for item in params:
            if item.name == 'rate_hz':
                self.rate_hz = max(0.05, float(item.value))
            elif item.name == 'timeout':
                self._client.timeout = float(item.value)
            elif item.name == 'model':
                self._client.model = str(item.value)
            elif item.name == 'max_side':
                self.max_side = int(item.value)
            elif item.name == 'jpeg_quality':
                self.jpeg_quality = int(item.value)
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------ callbacks
    def _on_image(self, msg: Image) -> None:
        try:
            bgr = image_msg_to_bgr(msg)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'图像解码失败: {exc}', throttle_duration_sec=5.0)
            return
        with self._lock:
            self._seq += 1
            self._frame = (self._seq, bgr, msg.header.stamp)
        self._new_frame.set()

    # ------------------------------------------------------------ worker
    def _loop(self) -> None:
        next_t = time.monotonic()
        while not self._stop.is_set():
            period = 1.0 / max(0.05, self.rate_hz)
            now = time.monotonic()
            if now < next_t:
                # 还没到下一次推理时刻：只等待，不推理（否则会被 10Hz 图像事件拖着跑）
                self._new_frame.wait(timeout=min(next_t - now, 0.1))
                continue
            if not self._new_frame.is_set():
                # 到点了但还没有新帧：等下一帧，最多等 0.1s
                self._new_frame.wait(timeout=0.1)
                continue
            self._new_frame.clear()
            with self._lock:
                frame = self._frame
            if frame is None:
                continue
            seq, bgr, stamp = frame
            self._infer(seq, bgr, stamp)
            next_t = time.monotonic() + period

    def _infer(self, seq: int, bgr, stamp) -> None:
        try:
            b64, _ = encode_jpeg_b64(bgr, max_side=self.max_side,
                                     quality=self.jpeg_quality)
            res = self._client.query(b64)
        except Exception as exc:  # noqa: BLE001
            self._n_fail += 1
            self.get_logger().error(f'推理异常: {exc}', throttle_duration_sec=5.0)
            self._publish_failure(stamp, str(exc))
            return

        if not res.ok:
            self._n_fail += 1
            if res.error == 'timeout' and not self._timeout_hinted:
                self._timeout_hinted = True
                self.get_logger().error(
                    f'推理超时（>{self._client.timeout:.0f}s）。CPU 推理单帧可达 15s+，'
                    '把超时调大：ros2 param set /vlm_obstacle_node timeout 120.0 '
                    '（换 GPU 后单帧约 0.5~1.5s，可再调回 30）')
            self._publish_failure(stamp, res.error, res.raw_text)
            return

        self._n_ok += 1
        self._last_ok = res
        alpha = min(1.0, max(0.0, self.risk_ema_alpha))
        smooth = (1.0 - alpha) * self._risk_ema + alpha * res.risk
        self._risk_ema = smooth

        msg = VlmObstacle()
        msg.header.stamp = stamp
        msg.header.frame_id = 'camera_link'
        msg.obstacle = bool(res.obstacle)
        msg.risk = float(smooth)
        msg.direction = res.direction
        msg.action = res.action
        msg.label = res.label
        msg.confidence = float(res.confidence)
        msg.stale = False
        msg.latency_ms = float(res.latency_ms)
        msg.raw_text = res.raw_text[:2000]
        self._pub.publish(msg)

        if self._pub_debug is not None:
            try:
                vis = draw_overlay(bgr, res.as_dict(), res.latency_ms)
                self._pub_debug.publish(bgr_to_image_msg(vis, stamp, 'camera_link'))
            except Exception:  # noqa: BLE001 - 调试图失败不影响主链路
                pass

        now = time.monotonic()
        if now - self._last_log > 3.0:
            self._last_log = now
            self.get_logger().info(
                f'VLM #{seq} obstacle={res.obstacle} risk={smooth:.2f} '
                f'dir={res.direction} act={res.action} label={res.label!r} '
                f'{res.latency_ms:.0f}ms ok={self._n_ok} fail={self._n_fail}')

    def _publish_failure(self, stamp, error: str, raw: str = '') -> None:
        """失败时也发布，但标 stale：下游必须能看到"我不知道"。"""
        msg = VlmObstacle()
        msg.header.stamp = stamp
        msg.header.frame_id = 'camera_link'
        msg.obstacle = False
        msg.risk = float(max(self._risk_ema, self.fail_publish_risk))
        msg.direction = 'none'
        msg.action = 'stop'
        msg.label = f'VLM 失败: {error}'[:120]
        msg.confidence = 0.0
        msg.stale = True
        msg.latency_ms = 0.0
        msg.raw_text = (raw or '')[:2000]
        self._pub.publish(msg)
        self.get_logger().warn(
            f'VLM 调用失败（连续 {self._n_fail} 次）: {error}', throttle_duration_sec=5.0)

    # ------------------------------------------------------------ lifecycle
    def destroy_node(self) -> bool:
        self._stop.set()
        self._new_frame.set()
        if self._worker.is_alive():
            self._worker.join(timeout=2.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VlmObstacleNode()
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
