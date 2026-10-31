# 参数速查与调参指南

## 0. 改参数只动 `tunable.json`

日常调参改这一个文件就够了：

```
src/vlm_avoid_bringup/config/tunable.json
```

它按节点分段，覆盖速度、障碍阈值、VLM 时序、深度相机外参、IMU：

```json
{
  "avoidance_controller": {
    "cruise_linear": 0.35,
    "obstacle_distance": 0.35,
    "stop_distance": 0.15,
    "lateral_swerve_max_time": 1.0,
    "lateral_swerve_max_angle": 0.4363
  },
  "vlm_obstacle_node": { "max_side": 384, "rate_hz": 1.5 },
  "depth_sectors_node": { "camera_height": 0.16, "camera_pitch": 0.0873 },
  "imu_state_node": { "snap_threshold": 0.5 }
}
```

- **优先级**：`avoidance.yaml` < `tunable.json` < launch 参数（`vlm_url`/`vlm_model`/话题等）。
- **生效方式**：重启 launch。colcon 用的是 `--symlink-install`，直接改 `src/` 下的文件即可。
- **写错键名不会报错**，只会被静默忽略。改完先跑：

```bash
python3 -m pytest tests/test_tunable_json.py -q   # 检查键名/类型/取值范围
```

> 其余参数在 [`avoidance.yaml`](../src/vlm_avoid_bringup/config/avoidance.yaml)（作为兜底默认值）。
> 标 ✅ 的可 `ros2 param set` 热改立即生效；标 ❄ 的需要重启节点。

## 1. 当前预设（已写进 yaml，属于"贴得近、跑得快"）

| 项目 | 值 | 含义 |
|---|---|---|
| 巡航 / 硬上限 | **0.35 / 0.45 m/s** | 直行速度 |
| 开始转向 | **0.35 m**（或几何风险 ≥ 0.70） | 比原来的 1.0 m 近了很多 |
| 硬停车 | **0.15 m** | 再近就停 |
| 滞回解除 | **0.60 m** | 退到这么远才恢复直行 |
| 避让角速度 | **0.85 rad/s** | 原地转弯半径 ≈ v/ω |
| 侧向障碍 | **只转向不减速** | 前方 >0.55 m 时，侧墙不再让车蠕动 |
| VLM | **1.5 Hz，max_side 384，num_predict 96，keep_alive 30m** | 单帧约 1.0~1.3 s |

## 2. 全参数表

### 2.1 速度

| 参数 | 默认 | 建议范围 | 作用 |
|---|---|---|---|
| `max_linear` | 0.45 | 0.1~0.6 | 硬上限 |
| `cruise_linear` | 0.35 | 0.1~0.5 | 巡航速度 |
| `creep_linear` | 0.12 | 0.05~0.25 | 避让时的速度上限 |
| `stale_creep` | 0.12 | 0.05~0.2 | VLM 掉线、仅深度时 |
| `recovery_linear` | -0.06 | -0.1~0 | 自救倒车 |
| `max_angular` | 1.20 | 0.5~2.0 | 角速度硬上限 |
| `avoid_angular` | 0.85 | 0.4~1.2 | 避让角速度基准 |

### 2.2 障碍判定

| 参数 | 默认 | 作用 |
|---|---|---|
| `stop_distance` | 0.15 | 净空 ≤ 此值 → 速度归零 |
| `slow_distance` | 0.80 | 几何风险公式的参考点 |
| `obstacle_distance` | 0.35 | 净空 < 此值 → 触发转向（滞回起点） |
| `release_margin` | 0.25 | 解除距离 = 0.35+0.25 = 0.60 |
| `turn_min_clearance` | 0.35 | 左右都小于它 → 无安全侧 → STOP |
| `side_margin` | 0.12 | 左右净空差多少才算"这边更空" |
| `side_push_gain` / `max` / `distance` | 0.60 / 0.45 / 0.75 | 贴墙时的"回中线"推力 |
| `clear_path_distance` | 0.55 | 前方净空 > 此值 + 障碍是侧向 → 只转向不减速 |
| `lateral_swerve_max_angle` | 0.4363 rad（25°） | 一次侧向避让最多转多少就收手（防绕圈） |
| `lateral_swerve_max_time` | 1.0 s | 一次侧向避让最长时间 |
| `lateral_cooldown` | 2.5 s | 两次侧向避让之间的冷却 |
| `obstacle_risk_threshold` | 0.70 | 融合风险 ≥ 此值 → 视为障碍（正前方障碍另有一票否决） |
| `unknown_risk` | 0.35 | 完全没感知时假定风险 |
| `goal_tolerance` | 0.30 | 距目标多近算到达 |
| `goal_kp` / `goal_max_angular` | 1.40 / 0.90 | 目的地导航的航向增益与角速度上限 |
| `goal_slow_angle` | 0.90 | 航向误差超过它就先原地转，不前进 |
| `goal_slow_radius` | 1.20 | 距目标 1.2 m 内开始减速 |
| `prefer_side` | +1 | 两侧一样空时优先方向；-1 = 优先右转 |

### 2.2b 激光雷达

| 参数 | 默认 | 作用 |
|---|---|---|
| `front_angle_deg` | 40 | 正前方扇区总张角（±20°） |
| `side_angle_deg` | 100 | 左右扇区外边界 |
| `max_range` | 12.0 | 输出上限（更远的记成它） |
| `range_min` / `range_max` | 0.12 / 20.0 | 过滤无效回波 |

雷达与深度是逐项取 min 的，所以调 `max_range` 只会影响"远处怎么报"，不会让车更保守。

### 2.2c A* 全局规划

| 参数 | 默认 | 作用 |
|---|---|---|
| `inflation_radius` | 0.28 m | 障碍膨胀半径；太小会贴着墙过，太大会把门堵死 |
| `replan_period` | 2.0 s | 多久按当前位置重算一次路径 |
| `waypoint_tolerance` | 0.35 m | 离路点多近算"到达该路点"，换下一个 |
| `smooth` | true | 视线平滑，减少拐点（关掉会看到密集的格子路径） |
| `map_offset_x/y` | 1.5 / 1.5 | odom 原点对应的世界坐标（= 出生点）；launch 用 `x:=`/`y:=` 自动传 |

换世界后必须重新生成地图：

```bash
python3 scripts/make_map_from_world.py --world <world> --out <json>
```

### 2.3 VLM 与反应速度

| 参数 | 默认 | 热改 | 作用 |
|---|---|---|---|
| `rate_hz` | 1.5 | ✅ | 每秒最多推理几次；**实际上限 = 1/单帧耗时** |
| `max_side` | 384 | ✅ | 送模型前缩放长边，**对耗时影响最大** |
| `jpeg_quality` | 70 | ✅ | 编码质量 |
| `num_predict` | 96 | ✅ | 输出 token 上限（JSON 约 20~40 token 就够） |
| `num_ctx` | 4096 | ✅ | 上下文；图像 token 要占，低于 2048 会截断 |
| `timeout` | 60 | ✅ | 单次请求超时 |
| `keep_alive` | 30m | ✅ | 模型在显存驻留时间，避免 5~10 s 冷启动 |
| `temperature` | 0 | ✅ | 固定 0，要稳定 JSON |
| `risk_ema_alpha` | 0.5 | ✅ | 风险平滑（越小越稳、越迟钝） |

配套的时序（在控制器里）：

| 参数 | 默认 | 作用 |
|---|---|---|
| `hold_time` | 1.2 | 避让承诺窗口：一次避让至少执行这么久 |
| `vlm_risk_tau` | 2.5 | 风险衰减常数，实际取 `max(2.5, 2×推理延迟)` |
| `vlm_stale_timeout` | 3.0 | 陈旧阈值，实际取 `max(3.0, 3×推理延迟)` |
| `publish_rate_hz` | 20 ❄ | 控制周期（50 ms） |

### 2.4 降级

| 参数 | 默认 | 作用 |
|---|---|---|
| `stale_creep` | 0.12 | 感知掉线后的爬行速度 |
| `stop_after_stale` | 6.0 | 掉线超过这么久就停 |
| `sensor_timeout` | 0.5 | 深度/IMU 多久算失效 |

### 2.5 IMU

| 参数 | 默认 | 作用 |
|---|---|---|
| `tilt_estop` | 0.90 | 倾角超过即 ESTOP (rad) |
| `heading_hold` | true | 直行时用航向保持 |
| `heading_kp` / `max_angular` | 0.9 / 0.35 | 航向修正增益 / 上限 |
| `heading_deadband` | 0.04 | 误差小于此值不修 |
| `heading_relock` | 0.60 | 偏差超过此值重新锁定目标航向 |

### 2.6 平滑与自救

| 参数 | 默认 | 作用 |
|---|---|---|
| `smoothing_alpha` | 0.45 | 指令一阶低通；越大越灵敏 |
| `recovery_after` | 4.0 | 停多久算卡死 |
| `recovery_time` | 2.0 | 自救持续多久 |
| `recovery_angular` | 0.80 | 自救角速度 |

## 3. 公式（改参数前先看联动）

```
risk_vlm = vlm_risk × exp(-age / max(vlm_risk_tau, 2×latency))
geo_risk = clamp((slow_distance - center) / (slow_distance - stop_distance))
risk     = max(risk_vlm, geo_risk)          # 侧向障碍时只用 geo_risk
risk     = max(risk, 0.7)                   # 模型说"正前方"时

obstacle = risk ≥ obstacle_risk_threshold
        或 center < obstacle_distance（滞回：退到 +release_margin 才解除）
        或 模型说 direction=center

v = min(cruise_linear,
        (center - stop_distance) × 0.9,
        cruise_linear × (1 - 0.85×risk))
避让时：v = min(v, creep_linear)   # 侧向障碍且前方通畅时不减速
ω = avoid_angular × (0.55 + 0.45×min(1, risk/0.8))
```

两个容易忽略的联动：

- **`obstacle_risk_threshold` 与 `slow_distance` 一起决定"多近开始躲"**：
  几何触发距离 ≈ `slow_distance - threshold × (slow_distance - stop_distance)`。
  默认 0.80 − 0.70×0.65 ≈ **0.34 m**，和 `obstacle_distance` 基本重合。
- **`cruise_linear` 与 `avoid_angular` 决定转弯半径**：`R ≈ v/ω`，
  默认 0.35/0.85 ≈ 0.41 m。走廊里障碍占一半宽度时，转弯半径太大会擦碰。
- **侧向避让必须"可执行"**：模型报 `direction=left/right` 时，只有同时满足
  ① 前方净空 > `clear_path_distance`、② 深度佐证那一侧确实更窄、
  ③ 还没用完"转角/时间预算"，才会真的转向并保持巡航速度；
  否则这条报告被完全忽略（既不转向也不减速）。否则 3B 模型对侧墙的误报
  会让车以恒定角速度一直转，画出一个 R≈0.7 m 的圆。

## 4. 症状 → 参数

| 症状 | 调什么 |
|---|---|
| 车走得太慢 / 老停 | `obstacle_distance` ↓、`stop_distance` ↓、`clear_path_distance` ↓ |
| 差点撞上 | `obstacle_distance` ↑、`stop_distance` ↑、`cruise_linear` ↓ |
| 目标在墙后不绕 / 顶墙 | 没有 `/plan` 或地图不匹配：确认 `path_planner_node` 在跑、`/map` 有数据；改过世界要重新生成地图 |
| **原地/大圈绕圈** | 侧向避让预算太宽 → `lateral_swerve_max_angle` ↓（0.4363→0.26）、`lateral_swerve_max_time` ↓（1.0→0.6）、`side_push_max` ↓；或 `clear_path_distance` ↓ 让侧向报告更容易被忽略 |
| 在两侧墙之间左右横跳 | `release_margin` ↑、`hold_time` ↑、`side_margin` ↑ |
| 贴着墙走 / 擦墙 | `side_push_gain` ↑、`side_push_distance` ↑ |
| 转弯幅度太大、画龙 | `avoid_angular` ↓、`smoothing_alpha` ↓ |
| 模型误报（通畅时说有障碍） | 调 prompt、`obstacle_risk_threshold` ↑、`risk_ema_alpha` ↓ |
| 模型反应慢 | `max_side` ↓、`num_predict` ↓、`rate_hz` ↓（别超 `1/延迟`） |
| 卡死后原地磨 | `recovery_time` ↑、`recovery_angular` ↑、`turn_min_clearance` ↓ |

## 5. 三套预设

**保守（真机首次上路）**
```yaml
max_linear: 0.20
cruise_linear: 0.15
stop_distance: 0.35
obstacle_distance: 0.60
obstacle_risk_threshold: 0.50
creep_linear: 0.08
```

**当前默认（仿真/熟练后）** —— 见第 1 节。

**激进（空旷场地刷速度）**
```yaml
max_linear: 0.60
cruise_linear: 0.50
stop_distance: 0.20
obstacle_distance: 0.40
avoid_angular: 1.00
rate_hz: 2.0
max_side: 320
```

## 6. 热改命令

```bash
# 阈值
ros2 param set /avoidance_controller obstacle_distance 0.45
ros2 param set /avoidance_controller stop_distance 0.20
ros2 param set /avoidance_controller obstacle_risk_threshold 0.60

# 速度
ros2 param set /avoidance_controller cruise_linear 0.30
ros2 param set /avoidance_controller avoid_angular 0.90

# VLM 提速
ros2 param set /vlm_obstacle_node max_side 320
ros2 param set /vlm_obstacle_node num_predict 64
ros2 param set /vlm_obstacle_node rate_hz 2.0

# 看当前值（热改不会写回 yaml，重启后恢复）
ros2 param get /avoidance_controller obstacle_distance
ros2 param dump /avoidance_controller | head -40
```

`depth_sectors_node` 的参数（`camera_height` / `camera_pitch` / `horizontal_fov` /
`ground_clearance`）**只能重启生效**。

## 7. 改动后的验证

```bash
bash scripts/run_tests.sh        # 58 个单测 + 端到端集成
python3 scripts/probe.py         # 看 v/ω 与 state 是否按预期变化
python3 scripts/doctor.py        # 看各通道频率与最终结论
```

改得很激进时，集成测试里 `v<=0.12`、`|ω|>=0.3` 这类断言可能需要同步调整
（它们对应的是默认预设）。
