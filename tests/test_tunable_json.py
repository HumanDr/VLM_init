"""校验 tunable.json：键名必须是真实存在的参数，且控制器的调参项不能漏。

这类配置最容易踩的坑是**拼写错误**——写错的键不会报错，只会被节点静默忽略，
然后你改了半天发现没效果。这个测试就是拦这个的。
"""
import json
import os
from dataclasses import fields

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
JSON_PATH = os.path.join(ROOT, 'src/vlm_avoid_bringup/config/tunable.json')

pytestmark = pytest.mark.skipif(not os.path.isfile(JSON_PATH),
                                reason='tunable.json 不存在')

KNOWN = {
    'avoidance_controller': {
        'publish_rate_hz', 'cmd_vel_topic', 'vlm_topic', 'sector_topic',
        'imu_state_topic', 'status_topic', 'enable', 'require_imu',
    },
    'vlm_obstacle_node': {
        'image_topic', 'result_topic', 'annotated_topic', 'vlm_url', 'api_style',
        'model', 'rate_hz', 'keep_alive', 'max_side', 'jpeg_quality', 'timeout',
        'num_predict', 'num_ctx', 'temperature', 'risk_ema_alpha',
        'publish_debug_image', 'fail_publish_risk',
    },
    'depth_sectors_node': {
        'depth_topic', 'result_topic', 'max_range', 'percentile',
        'min_valid_ratio', 'stride', 'camera_height', 'camera_pitch',
        'ground_clearance', 'horizontal_fov', 'vertical_fov',
        'left_band', 'center_band', 'right_band',
    },
    'imu_state_node': {
        'imu_topic', 'state_topic', 'alpha', 'snap_threshold', 'rate_hz', 'timeout',
    },
}


def _load():
    with open(JSON_PATH, encoding='utf-8') as f:
        return json.load(f)


def test_json_is_valid_and_has_all_sections():
    data = _load()
    assert isinstance(data, dict)
    for node in KNOWN:
        assert node in data, f'tunable.json 缺少节点段 {node}'
        assert isinstance(data[node], dict)


def test_keys_are_known_parameters():
    planner = pytest.importorskip('vlm_control.planner')
    planner_fields = {f.name for f in fields(planner.PlannerParams)}
    data = _load()
    for node, section in data.items():
        if node.startswith('_'):
            continue
        allowed = (KNOWN[node] | planner_fields
                   if node == 'avoidance_controller' else KNOWN[node])
        unknown = {k for k in section if not k.startswith('_')} - allowed
        assert not unknown, f'{node} 里有不存在的参数（拼写错误？）: {sorted(unknown)}'


def test_planner_section_covers_all_planner_params():
    """PlannerParams 新增字段后，必须同步补进 tunable.json。"""
    planner = pytest.importorskip('vlm_control.planner')
    names = {f.name for f in fields(planner.PlannerParams)}
    section = {k for k in _load()['avoidance_controller'] if not k.startswith('_')}
    missing = names - section
    assert not missing, f'tunable.json 的 avoidance_controller 缺少: {sorted(missing)}'


def test_value_types_are_sane():
    data = _load()
    c = data['avoidance_controller']
    assert 0.0 <= c['cruise_linear'] <= c['max_linear'] <= 1.0
    assert 0.0 < c['stop_distance'] < c['obstacle_distance'] < c['slow_distance']
    assert 0.0 < c['lateral_swerve_max_angle'] <= 1.0
    assert 0.0 < c['lateral_swerve_max_time'] <= 3.0
    assert c['smoothing_alpha'] <= 1.0
    v = data['vlm_obstacle_node']
    assert 128 <= v['max_side'] <= 1024
    assert v['timeout'] >= 5.0
    d = data['depth_sectors_node']
    assert d['camera_height'] >= 0.0
    assert 0.0 <= d['ground_clearance'] < 1.0
