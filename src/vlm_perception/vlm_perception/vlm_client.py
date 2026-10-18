"""Qwen-VL 客户端：把一张 JPEG 变成结构化的障碍物判断。

支持两种后端：
  * api_style='ollama'  -> http://127.0.0.1:11434/api/chat   （推荐，8GB 显存友好）
  * api_style='openai'  -> http://127.0.0.1:8000/v1/chat/completions （vLLM / LMDeploy）

本模块不依赖 rclpy，便于离线单测。
"""
from __future__ import annotations

import json
import re
import time
from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Tuple

import requests

# ---------------------------------------------------------------- prompts

SYSTEM_PROMPT = (
    '你是移动机器人的视觉感知模块。机器人正在室内向前行驶，'
    '你要判断当前画面是否会导致碰撞，并给出最安全的动作。'
    '只输出一个 JSON 对象，不要输出任何解释、前后缀或 markdown 代码块。'
)

USER_PROMPT = (
    '分析这张机器人前置相机的画面，按下面的 JSON 格式回答：\n'
    '{"obstacle": true/false, "risk": 0.0-1.0, '
    '"direction": "left|center|right|none", '
    '"action": "go_straight|turn_left|turn_right|stop", '
    '"label": "障碍物的简短中文描述", "confidence": 0.0-1.0}\n'
    '判断规则：\n'
    '1. 只考虑"会挡住你继续前进"的实体（前方的箱子、椅子、行人、垃圾筒、正对的长墙等）。\n'
    '   两侧的墙、走廊侧壁、地面、天花板、远处背景都不算障碍物——你是在走廊里行驶。\n'
    '2. risk 表示"保持当前直行会撞上的可能性"，不是"离得近"。'
    '只在画面左/右边缘出现的侧向物体，risk 不要超过 0.4。\n'
    '3. direction 是障碍物相对画面中线的位置：left/center/right，无遮挡填 none。\n'
    '4. action 必须与 direction 一致：障碍在左就 turn_right，在右就 turn_left，'
    '在正前方且两侧都看不清就 stop。\n'
    '5. 只有"正前方确实被挡住"时才把 risk 给到 0.7 以上。\n'
)

VALID_DIRECTIONS = ('left', 'center', 'right', 'none')
VALID_ACTIONS = ('go_straight', 'turn_left', 'turn_right', 'stop')

_DIRECTION_ALIASES = {
    'left': 'left', 'l': 'left', 'left_side': 'left', '左': 'left', '左边': 'left',
    'right': 'right', 'r': 'right', 'right_side': 'right', '右': 'right', '右边': 'right',
    'center': 'center', 'centre': 'center', 'middle': 'center', 'ahead': 'center',
    'front': 'center', '中': 'center', '中间': 'center', '正前': 'center',
    'none': 'none', 'clear': 'none', '': 'none', 'null': 'none', '无': 'none',
}
_ACTION_ALIASES = {
    'go_straight': 'go_straight', 'forward': 'go_straight', 'straight': 'go_straight',
    'continue': 'go_straight', 'none': 'go_straight', '': 'go_straight',
    '前进': 'go_straight', '直行': 'go_straight',
    'turn_left': 'turn_left', 'left': 'turn_left', 'steer_left': 'turn_left',
    '左转': 'turn_left',
    'turn_right': 'turn_right', 'right': 'turn_right', 'steer_right': 'turn_right',
    '右转': 'turn_right',
    'stop': 'stop', 'brake': 'stop', 'halt': 'stop', '停车': 'stop', '停止': 'stop',
}


@dataclass
class VlmResult:
    """一次推理的规范化结果。ok=False 表示本次调用失败，字段为兜底值。"""

    obstacle: bool = False
    risk: float = 0.0
    direction: str = 'none'
    action: str = 'go_straight'
    label: str = ''
    confidence: float = 0.0
    latency_ms: float = 0.0
    raw_text: str = ''
    ok: bool = True
    error: str = ''
    meta: Dict[str, Any] = field(default_factory=dict)

    def as_dict(self) -> Dict[str, Any]:
        return {
            'obstacle': self.obstacle, 'risk': self.risk,
            'direction': self.direction, 'action': self.action,
            'label': self.label, 'confidence': self.confidence,
        }


# ---------------------------------------------------------------- parsing

def _strip_code_fence(text: str) -> str:
    text = text.strip()
    text = re.sub(r'^```(?:json|JSON)?\s*', '', text)
    text = re.sub(r'\s*```$', '', text)
    return text.strip()


def extract_json(text: str) -> Optional[Dict[str, Any]]:
    """从模型输出里尽力挖出第一个完整 JSON 对象。"""
    if not text:
        return None
    candidate = _strip_code_fence(text)
    try:
        obj = json.loads(candidate)
        return obj if isinstance(obj, dict) else None
    except json.JSONDecodeError:
        pass
    # 退一步：括号配对扫描（模型常加解释性前缀）
    start = candidate.find('{')
    while start != -1:
        depth = 0
        in_str = False
        esc = False
        for i in range(start, len(candidate)):
            ch = candidate[i]
            if in_str:
                if esc:
                    esc = False
                elif ch == '\\':
                    esc = True
                elif ch == '"':
                    in_str = False
                continue
            if ch == '"':
                in_str = True
            elif ch == '{':
                depth += 1
            elif ch == '}':
                depth -= 1
                if depth == 0:
                    try:
                        obj = json.loads(candidate[start:i + 1])
                        if isinstance(obj, dict):
                            return obj
                    except json.JSONDecodeError:
                        break
        start = candidate.find('{', start + 1)
    return None


def _to_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return value >= 0.5
    if isinstance(value, str):
        return value.strip().lower() in ('true', 'yes', '1', '是', '有', '存在')
    return False


def _to_float(value: Any, default: float = 0.0) -> float:
    try:
        v = float(value)
    except (TypeError, ValueError):
        return default
    if v != v:  # NaN
        return default
    return v


def _clamp01(v: float) -> float:
    return max(0.0, min(1.0, v))


def normalize_result(obj: Dict[str, Any]) -> VlmResult:
    """容错地把模型 JSON 转成 VlmResult：别名、clamp、方向/动作一致性。"""
    if not isinstance(obj, dict):
        raise ValueError('result must be a dict')

    direction = str(obj.get('direction', 'none') or 'none').strip().lower()
    direction = _DIRECTION_ALIASES.get(direction, 'none')
    action = str(obj.get('action', 'go_straight') or 'go_straight').strip().lower()
    action = _ACTION_ALIASES.get(action, 'go_straight')

    obstacle = _to_bool(obj.get('obstacle'))
    risk = _clamp01(_to_float(obj.get('risk'), 0.0))
    confidence = _clamp01(_to_float(obj.get('confidence'), 0.0))
    if confidence == 0.0 and obstacle:
        confidence = 0.5  # 模型没给置信度但报了障碍，按中等置信处理

    # 一致性修正：方向与动作必须匹配，否则以下游安全为准
    if obstacle:
        if direction == 'left' and action in ('go_straight', 'turn_left'):
            action = 'turn_right'
        if direction == 'right' and action in ('go_straight', 'turn_right'):
            action = 'turn_left'
        if direction == 'center' and action == 'go_straight':
            action = 'stop'  # 正前方有障碍却想直行：最保守处理
        if direction == 'none':
            direction = 'center'  # 报了障碍但没给方位，按正前方处理
        risk = max(risk, 0.5)
    else:
        # 模型自相矛盾（说没障碍却给高风险）时，不触发避让，但也不当成完全通畅
        direction, action = 'none', 'go_straight'
        risk = min(risk, 0.3)

    label = str(obj.get('label', '') or '').strip()[:120]
    return VlmResult(obstacle=obstacle, risk=risk, direction=direction,
                     action=action, label=label, confidence=confidence)


# ---------------------------------------------------------------- client

class VlmClient:
    """同步 HTTP 客户端；调用方负责放到线程里，避免阻塞 ROS executor。"""

    def __init__(self, base_url: str = 'http://127.0.0.1:11434',
                 model: str = 'qwen2.5vl:3b',
                 api_style: str = 'ollama',
                 timeout: float = 10.0,
                 num_predict: int = 128,
                 num_ctx: int = 4096,
                 keep_alive: str = '30m',
                 temperature: float = 0.0,
                 system_prompt: str = SYSTEM_PROMPT,
                 user_prompt: str = USER_PROMPT):
        self.base_url = base_url.rstrip('/')
        self.model = model
        self.api_style = api_style.lower()
        if self.api_style not in ('ollama', 'openai'):
            raise ValueError(f'unsupported api_style: {api_style}')
        self.timeout = float(timeout)
        self.num_predict = int(num_predict)
        self.num_ctx = int(num_ctx)
        self.keep_alive = str(keep_alive)
        self.temperature = float(temperature)
        self.system_prompt = system_prompt
        self.user_prompt = user_prompt
        self._session = requests.Session()

    # -- public ----------------------------------------------------
    def health(self) -> Tuple[bool, str]:
        """探测后端是否在线；返回 (ok, detail)。"""
        try:
            if self.api_style == 'ollama':
                r = self._session.get(f'{self.base_url}/api/tags', timeout=min(3.0, self.timeout))
                r.raise_for_status()
                names = [m.get('name', '') for m in r.json().get('models', [])]
                hit = any(self.model.split(':')[0] in n for n in names)
                return True, f'ollama ok, models={names[:6]}{"" if hit else " (model not pulled yet)"}'
            r = self._session.get(f'{self.base_url}/v1/models', timeout=min(3.0, self.timeout))
            r.raise_for_status()
            return True, f'openai-compat ok: {r.text[:160]}'
        except Exception as exc:  # noqa: BLE001
            return False, f'{type(exc).__name__}: {exc}'

    def query(self, image_b64: str) -> VlmResult:
        t0 = time.perf_counter()
        try:
            if self.api_style == 'ollama':
                raw = self._query_ollama(image_b64)
            else:
                raw = self._query_openai(image_b64)
        except requests.Timeout:
            return VlmResult(ok=False, error='timeout', latency_ms=(time.perf_counter() - t0) * 1e3)
        except requests.RequestException as exc:
            return VlmResult(ok=False, error=f'http: {exc}',
                             latency_ms=(time.perf_counter() - t0) * 1e3)
        except Exception as exc:  # noqa: BLE001
            return VlmResult(ok=False, error=f'{type(exc).__name__}: {exc}',
                             latency_ms=(time.perf_counter() - t0) * 1e3)

        latency_ms = (time.perf_counter() - t0) * 1e3
        obj = extract_json(raw)
        if obj is None:
            return VlmResult(ok=False, error='unparseable output', raw_text=raw,
                             latency_ms=latency_ms)
        try:
            res = normalize_result(obj)
        except Exception as exc:  # noqa: BLE001
            return VlmResult(ok=False, error=f'normalize: {exc}', raw_text=raw,
                             latency_ms=latency_ms)
        res.raw_text = raw
        res.latency_ms = latency_ms
        return res

    # -- backends --------------------------------------------------
    def _query_ollama(self, image_b64: str) -> str:
        payload = {
            'model': self.model,
            'stream': False,
            'format': 'json',
            'keep_alive': self.keep_alive,
            'messages': [
                {'role': 'system', 'content': self.system_prompt},
                {'role': 'user', 'content': self.user_prompt, 'images': [image_b64]},
            ],
            'options': {
                'temperature': self.temperature,
                'num_predict': self.num_predict,
                'num_ctx': self.num_ctx,
            },
        }
        r = self._session.post(f'{self.base_url}/api/chat', json=payload, timeout=self.timeout)
        r.raise_for_status()
        body = r.json()
        if 'message' in body:
            return str(body['message'].get('content', ''))
        return str(body.get('response', ''))

    def _query_openai(self, image_b64: str) -> str:
        payload = {
            'model': self.model,
            'temperature': self.temperature,
            'max_tokens': self.num_predict,
            'messages': [
                {'role': 'system', 'content': self.system_prompt},
                {'role': 'user', 'content': [
                    {'type': 'text', 'text': self.user_prompt},
                    {'type': 'image_url',
                     'image_url': {'url': f'data:image/jpeg;base64,{image_b64}'}},
                ]},
            ],
        }
        r = self._session.post(f'{self.base_url}/v1/chat/completions',
                               json=payload, timeout=self.timeout)
        r.raise_for_status()
        body = r.json()
        return str(body['choices'][0]['message']['content'])
