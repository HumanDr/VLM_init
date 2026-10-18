#!/usr/bin/env python3
"""Qwen-VL 自检：合成一张"走廊中间有个红箱子"的图，调用模型看它是否识别为障碍。

用途：装完模型先确认"模型能看图 + 会按 JSON 回答 + 延迟多少"，
避免把模型问题误判成 ROS 管线问题。

    python3 scripts/vlm_selftest.py --model qwen2.5vl:3b
    python3 scripts/vlm_selftest.py --url http://127.0.0.1:8000 --api-style openai
"""
from __future__ import annotations

import argparse
import base64
import json
import os
import sys
import time

import cv2
import numpy as np
import requests

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'vlm_perception'))

SYSTEM = ('你是移动机器人的视觉感知模块。只输出一个 JSON 对象，不要解释。')
USER = ('这是机器人前置相机画面。判断是否会影响向前行驶，输出 JSON：'
        '{"obstacle": true/false, "risk": 0.0-1.0, '
        '"direction": "left|center|right|none", '
        '"action": "go_straight|turn_left|turn_right|stop", '
        '"label": "简短中文描述", "confidence": 0.0-1.0}')


def make_scene(path: str, with_obstacle: bool = True) -> None:
    w, h = 640, 480
    img = np.zeros((h, w, 3), np.uint8)
    for y in range(h):
        v = int(60 + 40 * y / h)
        img[y, :] = (v, v, v + 4)
    cv2.rectangle(img, (0, int(h * 0.52)), (w, h), (92, 94, 98), -1)
    cv2.fillConvexPoly(img, np.array(
        [[0, h], [0, int(h * 0.12)], [int(w * 0.2), int(h * 0.34)],
         [int(w * 0.2), int(h * 0.86)]], np.int32), (74, 76, 80))
    cv2.fillConvexPoly(img, np.array(
        [[w, h], [w, int(h * 0.12)], [int(w * 0.8), int(h * 0.34)],
         [int(w * 0.8), int(h * 0.86)]], np.int32), (80, 82, 86))
    if with_obstacle:
        bw, bh = 190, 200
        x0, x1 = w // 2 - bw // 2, w // 2 + bw // 2
        y1 = int(h * 0.9)
        y0 = y1 - bh
        cv2.rectangle(img, (x0, y0), (x1, y1), (40, 40, 205), -1)
        cv2.fillConvexPoly(img, np.array(
            [[x0, y0], [x1, y0], [x1 - 30, y0 - 42], [x0 - 30, y0 - 42]],
            np.int32), (75, 75, 235))
        cv2.rectangle(img, (x0, y0), (x1, y1), (25, 25, 30), 2)
    cv2.imwrite(path, img)


def query_ollama(url: str, model: str, b64: str, timeout: float) -> str:
    payload = {
        'model': model, 'stream': False, 'format': 'json',
        'messages': [
            {'role': 'system', 'content': SYSTEM},
            {'role': 'user', 'content': USER, 'images': [b64]},
        ],
        'options': {'temperature': 0.0, 'num_predict': 128, 'num_ctx': 4096},
    }
    r = requests.post(f'{url}/api/chat', json=payload, timeout=timeout)
    r.raise_for_status()
    return r.json()['message']['content']


def query_openai(url: str, model: str, b64: str, timeout: float) -> str:
    payload = {
        'model': model, 'temperature': 0.0, 'max_tokens': 128,
        'messages': [
            {'role': 'system', 'content': SYSTEM},
            {'role': 'user', 'content': [
                {'type': 'text', 'text': USER},
                {'type': 'image_url', 'image_url': {'url': f'data:image/jpeg;base64,{b64}'}},
            ]},
        ],
    }
    r = requests.post(f'{url}/v1/chat/completions', json=payload, timeout=timeout)
    r.raise_for_status()
    return r.json()['choices'][0]['message']['content']


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--url', default='http://127.0.0.1:11434')
    ap.add_argument('--model', default='qwen2.5vl:3b')
    ap.add_argument('--api-style', choices=['ollama', 'openai'], default='ollama')
    ap.add_argument('--timeout', type=float, default=120.0)
    ap.add_argument('--out-dir', default='.')
    args = ap.parse_args()

    img_path = os.path.join(args.out_dir, 'vlm_selftest_scene.jpg')
    make_scene(img_path, with_obstacle=True)
    with open(img_path, 'rb') as f:
        b64 = base64.b64encode(f.read()).decode()

    print(f'  测试图: {img_path}')
    print(f'  模型  : {args.model}  ({args.url}, {args.api_style})')
    t0 = time.perf_counter()
    try:
        raw = (query_ollama if args.api_style == 'ollama' else query_openai)(
            args.url, args.model, b64, args.timeout)
    except Exception as exc:  # noqa: BLE001
        print(f'  ✗ 调用失败: {type(exc).__name__}: {exc}')
        print('    排查: ollama list / curl http://127.0.0.1:11434/api/tags')
        return 1
    dt = (time.perf_counter() - t0) * 1000
    print(f'  原始返回: {raw[:300]}')

    try:
        text = raw.strip().removeprefix('```json').removeprefix('```').removesuffix('```')
        obj = json.loads(text.strip())
    except Exception:  # noqa: BLE001
        print('  ✗ 返回不是合法 JSON（管线里的容错解析会兜底，但说明模型没按格式输出）')
        return 1

    print(f'  解析结果: {json.dumps(obj, ensure_ascii=False)}')
    print(f'  单帧延迟: {dt:.0f} ms')
    if not obj.get('obstacle'):
        print('  ⚠ 模型没把中间的红箱子判为障碍——语义能力偏弱或提示词与水印不匹配，'
              '建议换 qwen3-vl:4b 或调 config 里的 prompt')
        return 2
    print('  ✓ 视觉链路正常：能看图、能按 JSON 输出、判定为障碍')
    return 0


if __name__ == '__main__':
    sys.exit(main())
