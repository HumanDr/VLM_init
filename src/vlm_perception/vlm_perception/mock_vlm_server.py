"""Mock VLM 服务：用纯 Python HTTP 服务模拟 Ollama / OpenAI 兼容接口。

用途：
  * 没有 GPU / 还没下载 Qwen 时，先用它把 ROS2 全链路跑通；
  * CI / 端到端测试的确定性后端（根据画面里的彩色物体给固定答案）；
  * 对比"模型很慢 / 模型挂了"时控制器的降级行为。

启动：
    ros2 run vlm_perception mock_vlm_server --port 11435
    # 或 python3 -m vlm_perception.mock_vlm_server --port 11435 --delay-ms 800

它不追求"看懂"，只做颜色/饱和度统计，所以能复现：中间有障碍/左侧/右侧/通畅 四种场景。
"""
from __future__ import annotations

import argparse
import json
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict, Optional, Tuple

import numpy as np

try:  # 既支持 ros2 run，也支持 python -m
    from .image_utils import decode_b64_image
except ImportError:  # pragma: no cover
    from vlm_perception.image_utils import decode_b64_image  # type: ignore


# ---------------------------------------------------------------- 视觉判断

def _saturated_mask(bgr: np.ndarray) -> np.ndarray:
    """灰度背景上的彩色物体 -> 高饱和像素。走廊场景里这就是"障碍物"。"""
    f = bgr.astype(np.int16)
    mx = f.max(axis=2)
    mn = f.min(axis=2)
    sat = mx - mn
    return (sat > 45) & (mx > 55)


def analyze(bgr: np.ndarray) -> Dict[str, Any]:
    """按左/中/右三个竖条统计彩色占比，给出与 Qwen 同 schema 的答案。"""
    h, w = bgr.shape[:2]
    mask = _saturated_mask(bgr)
    bands = {
        'left': (int(w * 0.00), int(w * 0.33)),
        'center': (int(w * 0.33), int(w * 0.67)),
        'right': (int(w * 0.67), int(w * 1.00)),
    }
    frac = {}
    for name, (c0, c1) in bands.items():
        sub = mask[int(h * 0.25):, c0:c1]  # 忽略顶部 1/4（天花板/灯）
        frac[name] = float(sub.mean()) if sub.size else 0.0

    thresh = 0.02
    top = max(frac, key=frac.get)
    if frac[top] < thresh:
        return {'obstacle': False, 'risk': 0.03, 'direction': 'none',
                'action': 'go_straight', 'label': '通道通畅', 'confidence': 0.9}

    direction = top
    magnitude = min(1.0, frac[top] / 0.25)
    risk = round(min(0.95, 0.55 + 0.4 * magnitude), 3)
    if direction == 'center':
        action = 'stop' if frac['center'] > 0.18 else 'turn_right'
        label = '红色箱子挡住了通道'
    elif direction == 'left':
        action, label = 'turn_right', '左侧有彩色障碍物'
    else:
        action, label = 'turn_left', '右侧有彩色障碍物'
    return {'obstacle': True, 'risk': risk, 'direction': direction,
            'action': action, 'label': label, 'confidence': 0.85}


def _pick_images(payload: Dict[str, Any]) -> Optional[str]:
    """从两种 API 的请求体里取出第一张 base64 图。"""
    # Ollama: messages[].images
    for m in payload.get('messages', []) or []:
        imgs = m.get('images') if isinstance(m, dict) else None
        if imgs:
            return imgs[0]
    # OpenAI: content[].image_url.url = data:image/jpeg;base64,....
    for m in payload.get('messages', []) or []:
        content = m.get('content') if isinstance(m, dict) else None
        if isinstance(content, list):
            for part in content:
                if not isinstance(part, dict):
                    continue
                url = (part.get('image_url') or {}).get('url', '')
                if 'base64,' in url:
                    return url.split('base64,', 1)[1]
    # Ollama /api/generate: images
    imgs = payload.get('images')
    if imgs:
        return imgs[0]
    return None


class MockVlmHandler(BaseHTTPRequestHandler):
    server_version = 'MockVLM/1.0'
    opts: argparse.Namespace  # 由 make_server 注入

    def log_message(self, fmt: str, *args) -> None:  # 静音默认访问日志
        return

    # -------------------------------------------------- helpers
    def _send(self, code: int, obj: Dict[str, Any]) -> None:
        body = json.dumps(obj, ensure_ascii=False).encode('utf-8')
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _read_json(self) -> Dict[str, Any]:
        n = int(self.headers.get('Content-Length', '0') or 0)
        raw = self.rfile.read(n) if n else b'{}'
        try:
            obj = json.loads(raw.decode('utf-8'))
        except Exception:  # noqa: BLE001
            return {}
        return obj if isinstance(obj, dict) else {}

    def _answer(self, payload: Dict[str, Any]) -> Tuple[int, Dict[str, Any]]:
        o = self.opts
        o.n_requests += 1
        if o.delay_ms:
            time.sleep(o.delay_ms / 1000.0)
        if o.fail:
            return 500, {'error': 'mock failure injected'}

        if o.always_obstacle:
            result = {'obstacle': True, 'risk': 0.9, 'direction': 'center',
                      'action': 'stop', 'label': '强制障碍', 'confidence': 0.99}
        elif o.never_obstacle:
            result = {'obstacle': False, 'risk': 0.01, 'direction': 'none',
                      'action': 'go_straight', 'label': '强制通畅', 'confidence': 0.99}
        else:
            b64 = _pick_images(payload)
            if not b64:
                return 400, {'error': 'no image in request'}
            bgr = decode_b64_image(b64)
            if bgr is None:
                return 400, {'error': 'image decode failed'}
            result = analyze(bgr)
        if o.verbose:
            print(f'[mock-vlm] #{o.n_requests} -> {json.dumps(result, ensure_ascii=False)}',
                  file=sys.stderr, flush=True)
        return 200, result

    @staticmethod
    def _wrap(style: str, result: Dict[str, Any]) -> Dict[str, Any]:
        text = json.dumps(result, ensure_ascii=False)
        if style == 'openai':
            return {'id': 'mock', 'object': 'chat.completion',
                    'choices': [{'index': 0, 'message': {'role': 'assistant',
                                                         'content': text},
                                 'finish_reason': 'stop'}]}
        return {'model': 'mock-vlm', 'message': {'role': 'assistant', 'content': text},
                'done': True, 'eval_count': 32}

    # -------------------------------------------------- routes
    def do_GET(self) -> None:  # noqa: N802
        path = self.path.split('?')[0]
        if path == '/api/tags':
            self._send(200, {'models': [{'name': self.opts.model,
                                         'model': self.opts.model}]})
        elif path == '/v1/models':
            self._send(200, {'object': 'list', 'data': [{'id': self.opts.model}]})
        elif path in ('/health', '/'):
            self._send(200, {'status': 'ok', 'requests': self.opts.n_requests})
        else:
            self._send(404, {'error': 'not found'})

    def do_POST(self) -> None:  # noqa: N802
        path = self.path.split('?')[0]
        style = 'openai' if path.startswith('/v1/') else 'ollama'
        payload = self._read_json()
        code, result = self._answer(payload)
        if code != 200:
            self._send(code, result)
            return
        self._send(200, self._wrap(style, result))


def make_server(port: int = 11435, host: str = '127.0.0.1', **kwargs) -> ThreadingHTTPServer:
    opts = argparse.Namespace(
        always_obstacle=kwargs.get('always_obstacle', False),
        never_obstacle=kwargs.get('never_obstacle', False),
        fail=kwargs.get('fail', False),
        delay_ms=int(kwargs.get('delay_ms', 0)),
        model=kwargs.get('model', 'mock-vlm'),
        verbose=kwargs.get('verbose', False),
        n_requests=0,
    )
    handler = type('BoundHandler', (MockVlmHandler,), {'opts': opts})
    httpd = ThreadingHTTPServer((host, port), handler)
    httpd.daemon_threads = True
    httpd.mock_opts = opts  # type: ignore[attr-defined]
    return httpd


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description='Mock VLM 服务（Ollama / OpenAI 兼容）')
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=11435)
    ap.add_argument('--model', default='mock-vlm')
    ap.add_argument('--delay-ms', type=int, default=0, help='模拟推理延迟')
    ap.add_argument('--fail', action='store_true', help='总是返回 500（测降级）')
    ap.add_argument('--always-obstacle', action='store_true')
    ap.add_argument('--never-obstacle', action='store_true')
    ap.add_argument('--verbose', action='store_true')
    args = ap.parse_args(argv)

    httpd = make_server(args.port, args.host, always_obstacle=args.always_obstacle,
                        never_obstacle=args.never_obstacle, fail=args.fail,
                        delay_ms=args.delay_ms, model=args.model,
                        verbose=args.verbose)
    print(f'[mock-vlm] listening on http://{args.host}:{args.port} '
          f'(delay={args.delay_ms}ms fail={args.fail})', file=sys.stderr, flush=True)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        httpd.shutdown()


if __name__ == '__main__':
    main()
