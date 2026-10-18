"""感知层单测：JSON 容错解析、图像编解码、深度扇区、Mock 服务闭环。"""
import json
import math
import threading

import cv2
import numpy as np
import pytest

from vlm_perception.depth_sectors import compute_sectors
from vlm_perception.image_utils import (bgr_to_image_msg, decode_b64_image,
                                        encode_jpeg_b64, image_msg_to_array,
                                        image_msg_to_bgr, depth_msg_to_meters)
from vlm_perception.mock_vlm_server import analyze, make_server
from vlm_perception.vlm_client import (VlmClient, extract_json, normalize_result)


# ---------------------------------------------------------------- parsing

def test_extract_json_plain():
    assert extract_json('{"obstacle": true}') == {'obstacle': True}


def test_extract_json_fenced_and_chatty():
    text = '好的，结果如下：\n```json\n{"obstacle": false, "risk": 0.1}\n```\n以上。'
    assert extract_json(text) == {'obstacle': False, 'risk': 0.1}


def test_extract_json_broken():
    assert extract_json('not json at all') is None
    assert extract_json('{"obstacle": tru') is None
    assert extract_json('') is None


def test_normalize_aliases_and_consistency():
    r = normalize_result({'obstacle': 'true', 'risk': 1.7, 'direction': '左',
                          'action': 'forward', 'label': '箱子', 'confidence': 0.8})
    assert r.obstacle is True
    assert r.risk == 1.0                     # clamp
    assert r.direction == 'left'
    assert r.action == 'turn_right'          # 方向/动作一致性修正


def test_action_never_turns_into_obstacle():
    r = normalize_result({'obstacle': True, 'direction': 'left',
                          'action': 'turn_left', 'risk': 0.7})
    assert r.action == 'turn_right'
    r = normalize_result({'obstacle': True, 'direction': 'right',
                          'action': 'turn_right', 'risk': 0.7})
    assert r.action == 'turn_left'


def test_normalize_center_obstacle_forces_stop():
    r = normalize_result({'obstacle': True, 'direction': 'center',
                          'action': 'go_straight', 'risk': 0.6})
    assert r.action == 'stop'


def test_normalize_clear_forces_go_straight():
    r = normalize_result({'obstacle': False, 'direction': 'left',
                          'action': 'turn_left', 'risk': 0.9})
    assert (r.direction, r.action) == ('none', 'go_straight')
    assert r.risk <= 0.3          # 矛盾输出被压到低风险，但不假装"完全通畅"


def test_normalize_garbage_types():
    r = normalize_result({'obstacle': None, 'risk': 'high', 'direction': 3})
    assert r.obstacle is False and r.risk == 0.0 and r.direction == 'none'


# ---------------------------------------------------------------- images

def _red_box_image(w=320, h=240, cx=0.5, size=0.3):
    img = np.full((h, w, 3), 90, np.uint8)
    bw = int(w * size)
    x0, x1 = int(w * cx) - bw // 2, int(w * cx) + bw // 2
    cv2.rectangle(img, (x0, int(h * 0.4)), (x1, int(h * 0.9)), (40, 40, 205), -1)
    return img


def test_image_msg_roundtrip_rgb8():
    img = _red_box_image()
    msg = bgr_to_image_msg(img)
    back = image_msg_to_bgr(msg)
    assert back.shape == img.shape
    assert np.array_equal(back, img)


def test_image_msg_rgb8_swaps_to_bgr():
    img = _red_box_image()
    rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    msg = bgr_to_image_msg(rgb)
    msg.encoding = 'rgb8'
    back = image_msg_to_bgr(msg)
    assert np.array_equal(back, img)


def test_image_msg_respects_step_padding():
    img = np.full((4, 5, 3), 7, np.uint8)
    msg = bgr_to_image_msg(img)
    # 人为插入行尾 padding，验证不会撕列
    padded = bytearray()
    for row in range(4):
        padded += img[row].tobytes() + b'\x00\x00'
    msg.data = bytes(padded)
    msg.step = 5 * 3 + 2
    back = image_msg_to_bgr(msg)
    assert np.array_equal(back, img)


def test_encode_jpeg_b64_roundtrip_and_resize():
    img = _red_box_image(640, 480)
    b64, (h, w) = encode_jpeg_b64(img, max_side=320, quality=80)
    assert max(h, w) <= 320
    back = decode_b64_image(b64)
    assert back is not None and back.shape[0] == h


def test_depth_16uc1_mm_to_meters():
    depth_mm = np.full((3, 4), 1500, np.uint16)
    msg = bgr_to_image_msg(np.zeros((1, 1, 3), np.uint8))  # 占位
    msg.encoding = '16UC1'
    msg.height, msg.width = 3, 4
    msg.step = 8
    msg.data = np.ascontiguousarray(depth_mm).tobytes()
    meters = depth_msg_to_meters(msg)
    assert meters.shape == (3, 4)
    assert np.allclose(meters, 1.5)


# ---------------------------------------------------------------- depth sectors

def test_sectors_center_obstacle():
    d = np.full((240, 320), 5.0, np.float32)
    d[120:, 100:220] = 0.8
    s = compute_sectors(d)
    assert s['valid'] is True
    assert s['center'] == pytest.approx(0.8, abs=0.05)
    assert s['left'] > 2.0 and s['right'] > 2.0
    assert s['min_range'] == pytest.approx(0.8, abs=0.05)


def test_sectors_all_invalid():
    d = np.zeros((240, 320), np.float32)
    s = compute_sectors(d, max_range=5.0)
    assert s['valid'] is False
    assert s['center'] == 5.0


def _synthetic_corridor_depth(h=240, w=320, cam_h=0.16, pitch=0.0873, vfov=0.948):
    """按真实相机外参合成"地面 + 远处"的深度图（相机看得到脚下地板）。"""
    fy = (h / 2.0) / math.tan(vfov / 2.0)
    cy = (h - 1) / 2.0
    d = np.full((h, w), 5.0, np.float32)
    for v in range(h):
        beta = pitch + math.atan((v - cy) / fy)
        if beta > 0.01:
            d[v, :] = min(5.0, cam_h / math.sin(beta))
    return d


def test_floor_filtering_recovers_obstacle():
    """踩过的坑：不滤地面时，脚下地板（~0.5m）会被当成障碍，车永远不敢走。"""
    d = _synthetic_corridor_depth()
    d[90:128, 110:210] = 1.2        # 1.2m 处一个箱子
    raw = compute_sectors(d)
    filt = compute_sectors(d, camera_height=0.16, camera_pitch=0.0873,
                           vfov=0.948, ground_clearance=0.06)
    assert raw['center'] < 0.8, f'不滤地面应该被地板骗: {raw["center"]:.2f}'
    assert filt['center'] == pytest.approx(1.2, abs=0.3)
    assert filt['left'] > 3.0 and filt['right'] > 3.0


def test_floor_filtering_disabled_when_height_zero():
    d = _synthetic_corridor_depth()
    assert compute_sectors(d)['center'] == pytest.approx(
        compute_sectors(d, camera_height=0.0)['center'])


def test_sectors_ignores_nan_and_zeros():
    d = np.full((100, 100), np.nan, np.float32)
    d[50:, 40:60] = 0.0
    d[60:, 45:55] = 1.2
    s = compute_sectors(d, max_range=4.0)
    assert s['center'] == pytest.approx(1.2, abs=0.1)


# ---------------------------------------------------------------- mock server

def test_mock_analyze_center():
    out = analyze(_red_box_image())
    assert out['obstacle'] is True
    assert out['direction'] == 'center'


def test_mock_analyze_clear():
    img = np.full((240, 320, 3), 90, np.uint8)
    out = analyze(img)
    assert out['obstacle'] is False and out['direction'] == 'none'


@pytest.fixture()
def mock_server():
    httpd = make_server(port=0)
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    port = httpd.server_address[1]
    yield f'http://127.0.0.1:{port}'
    httpd.shutdown()
    httpd.server_close()


def test_client_against_mock_ollama(mock_server):
    client = VlmClient(base_url=mock_server, model='mock', api_style='ollama')
    ok, detail = client.health()
    assert ok, detail
    b64, _ = encode_jpeg_b64(_red_box_image())
    res = client.query(b64)
    assert res.ok, res.error
    assert res.obstacle is True
    assert res.direction == 'center'
    assert res.latency_ms > 0


def test_client_against_mock_openai(mock_server):
    client = VlmClient(base_url=mock_server, model='mock', api_style='openai')
    b64, _ = encode_jpeg_b64(_red_box_image(320, 240, cx=0.5))
    res = client.query(b64)
    assert res.ok, res.error
    assert res.obstacle is True


def test_client_reports_failure():
    httpd = make_server(port=0, fail=True)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    try:
        client = VlmClient(base_url=f'http://127.0.0.1:{httpd.server_address[1]}',
                           timeout=2.0)
        b64, _ = encode_jpeg_b64(_red_box_image())
        res = client.query(b64)
        assert res.ok is False
        assert res.error
    finally:
        httpd.shutdown()
        httpd.server_close()


def test_client_unreachable():
    client = VlmClient(base_url='http://127.0.0.1:1', timeout=1.0)
    res = client.query('AAAA')
    assert res.ok is False
    ok, _ = client.health()
    assert ok is False
