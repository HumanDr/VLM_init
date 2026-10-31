# VLM 视觉避障（ROS2 Humble + Gazebo Classic + Qwen-VL）

用**相机 + 深度 + 激光雷达 + IMU** 做一个"感知 → 决策 → 执行"闭环：相机图像送本地 Qwen-VL
做语义判断，深度相机和 360° 激光雷达做几何安全网，IMU 负责姿态/航向估计与倾覆急停，
决策结果变成 `/cmd_vel` 控制差速小车；并支持用 `/goal_pose` 指定**目的地**自动导航。

> 本仓库已在此机器上完成：**Gazebo 11.10.2 已安装**、6 个 ROS2 包构建通过、**100 个测试通过**、
> **1 个端到端集成测试通过**，并且 **Ollama + Qwen2.5-VL-3B 已装好并用真模型跑通了闭环**
> （CPU 推理下 VLM 判定"红色立方体" → 控制器 `AVOID_LEFT, v=0.10, w=+0.60`）。
> URDF/world 已通过 `gz sdf` 校验（4 个插件 + camera/depth/imu 三个传感器全部识别）。
> 唯一没法在这里做的是**打开 Gazebo 画面**：当前会话没有 GPU 设备节点，`gzserver` 建不出 GLX 上下文。

---

> **📖 想通读工程？看 [docs/project_guide.md](docs/project_guide.md)**
> —— 运行方式、参数手册、代码地图、初学者阅读路径、踩坑速查，一份就够。

## 0. 新手从这里开始

没接触过 ROS2 / 视觉大模型也没关系：
**[docs/beginner_guide.md](docs/beginner_guide.md)** 用"眼睛—翻译—司机"的比喻把
工程原理、关键代码、**推荐读代码顺序**讲了一遍，还配了 8 张图和 5 个动手实验。

## 1. 先跑起来（3 条命令）

```bash
cd ~/VLN_ws

# ① 装 Gazebo Classic 11 + ROS 插件（需要 sudo 密码；106 个包，约 121 MB）
bash scripts/install_gazebo.sh

# ② 装 Ollama 并拉取 Qwen-VL（两种方式二选一）
bash scripts/setup_vlm.sh              # A. 有 sudo：官方安装脚本 + systemd 服务
bash scripts/install_ollama_noroot.sh  # B. 没 sudo：装到 ~/.local/bin + ~/.ollama，任意终端可用

# 之后每次开始/结束实验，手动控制 Ollama（不随仿真自动启停）：
bash scripts/ollama_serve.sh           # 启动（幂等）
bash scripts/ollama_stop.sh            # 停止，释放显存（幂等）

# ③ 构建 + 跑测试（不需要 GPU，也不需要 Gazebo）
bash scripts/build.sh
bash scripts/run_tests.sh

# ④ 仿真：仓库 + SLAM + 避障 + rviz2
#    默认 localization:=rtabmap（RGB-D + 激光 + IMU）；
#    没装 rtabmap 就用 localization:=slam（仓库内置，零依赖）；
#    要对比"真值定位"用 localization:=ground_truth
bash scripts/run_sim.sh

# ⑤ 给个目的地，让它自己走过去（另开终端）
bash scripts/goal.sh 8 5          # 先试近的：开放区域，路上只有红箱
bash scripts/goal.sh 22 -6        # 远的：中间有两道隔墙，考验"沿墙绕行"的极限
bash scripts/goal.sh cancel       # 取消，恢复自主巡航
```

也可以在 rviz2 里用工具栏的 **2D Goal Pose** 直接点地面（会同时在 rviz 里画出 A* 路径）。

目标在墙后面也能绕：`path_planner_node` 用 `warehouse_map.json` 做 A* 全局规划，
控制器逐点跟随。地图由世界生成，**改了世界要重新生成**：

```bash
python3 scripts/make_map_from_world.py          # 默认 warehouse
# 换走廊场景时，地图和世界要配套：
python3 scripts/make_map_from_world.py --world src/vlbot_gazebo/worlds/corridor.world \
    --out src/vlm_avoid_bringup/config/corridor_map.json
bash scripts/run_sim.sh world:=...corridor.world map_file:=.../corridor_map.json
```

调试三件套：

```bash
python3 scripts/doctor.py     # 车不动？一键诊断（读 5 秒后给结论）
python3 scripts/probe.py      # 实时看 状态/风险/净空/cmd_vel/VLM判断
bash scripts/teleop.sh        # 手动遥控，验证底盘本身能动能转
```

安装细节与国内网络注意事项见 [docs/setup.md](docs/setup.md)：
Ollama 官方包在国内直连约 95 KB/s，脚本会自动改用加速镜像并强制校验 sha256。

遇到问题先看 [新手导读的 FAQ](docs/beginner_guide.md#9-常见疑问-faq)，
里面有"车不动 / 转圈 / VLM 误报"的标准排查路径。

**什么都不能装时的保底方案**（无 Gazebo、无 GPU 也能看逻辑跑通）：

```bash
bash scripts/run_mock_demo.sh
# 另开终端切场景，观察 /cmd_vel 变化
ros2 topic pub -1 /test/scene std_msgs/String "{data: center}"
ros2 topic echo /vlm_avoid/status
```

---

## 2. 为什么这样设计：VLM 慢，避障要快

直接让大模型"看图出速度"是跑不起来的：3B 模型在 4060 上单帧 0.6~1.5 s，
而避障控制回路需要 20 Hz。所以这里做了**双通道 + 承诺窗口**：

| 通道 | 频率 | 职责 | 失效时 |
|---|---|---|---|
| VLM 语义通道（Qwen-VL） | 1~2 Hz | "那是什么、该往哪让"（语义/可解释） | 标记 `stale`，降级 |
| 深度几何通道（可选 RGB-D） | 10~15 Hz | "离撞上还有多少米"（实时防撞兜底，**会按相机外参剔除地面点**） | 退化为纯 VLM+IMU |
| IMU 状态估计 | 30 Hz | 航向保持（走直线不跑偏）、倾覆急停 | 停车（`require_imu:=true` 时） |
| 反应式规划器 | 20 Hz | 融合上面三路，输出 `(v, ω)`，含状态机/平滑/看门狗 | — |

关键机制：

1. **推理在独立线程**，图像回调只存"最新帧"；单帧在途时**跳过**新帧，避免请求堆积导致延迟雪崩。
2. **承诺窗口（hold_time=1.2s）**：VLM 报一次"前方有障碍"后，即使下一秒它说"通畅"，
   也把避让动作执行完——解决 1 Hz 推理带来的抖动/犹豫。
3. **风险时间衰减**：`risk * exp(-age/τ)`，模型"上一帧看到的"不会永远生效。
4. **风险融合取 max**：语义风险与几何风险谁大听谁的。
5. **侧向推离**：左侧贴墙（0.45 m）但正前方通畅时不做急避让，只轻微回中线，避免"见墙就拐"。
6. **航向保持**：IMU 积分航向 + 比例修正，直行不跑偏；避让结束时重新锁定目标航向，不把车头掰回去。
7. **看门狗**：VLM 掉线 → 有深度就 0.7 倍速继续；没深度就先蠕行 6 s 再停；
   从没收到过任何感知 → 直接停（绝不"盲开"）。
8. **退出时主动发零速度**（真机防"节点死了车还在跑"）。

---

## 3. 架构

```mermaid
flowchart LR
  CAM["/camera/image_raw<br/>640x480 @10Hz"] --> VLM["vlm_obstacle_node<br/>工作线程 1Hz"]
  VLM <-->|HTTP /api/chat| QWEN[("Ollama<br/>qwen2.5vl:3b")]
  CAM --> DEEP["/camera/depth/image_raw<br/>320x240 @10Hz"] --> SEC["depth_sectors_node<br/>左/中/右扇区"]
  LID["/scan<br/>360° @10Hz"] --> SCAN["scan_sectors_node<br/>左/中/右扇区"]
  IMU["/imu/data<br/>100Hz"] --> FILT["imu_state_node<br/>互补滤波 30Hz"]
  VLM -->|"/perception/vlm/result<br/>VlmObstacle"| CTRL["avoidance_controller<br/>20Hz 反应式规划器"]
  SEC -->|"/perception/sectors/result<br/>SectorScan"| CTRL
  SCAN -->|"/perception/scan_sectors<br/>SectorScan"| CTRL
  FILT -->|"/state/imu<br/>ImuState"| CTRL
  ODOM["/odom（真值）"] --> CTRL
  GOAL["/goal_pose<br/>目的地"] --> PLAN["path_planner_node<br/>A* 全局规划（含膨胀）"]
  ODOM --> PLAN
  MAPF[("warehouse_map.json<br/>26x18m 栅格图")] --> PLAN
  PLAN -->|"/plan 路径"| CTRL
  PLAN -->|"/map 可视化"| RVZ["rviz2"]
  CTRL -->|"Twist"| CMD["/cmd_vel"]
  CTRL -->|"DriveStatus"| DBG["/vlm_avoid/status"]
  VLM -->|"叠加图"| ANN["/perception/vlm/annotated"]
```

包结构：

```
src/
├── vlm_avoid_interfaces/   4 个消息：VlmObstacle / SectorScan / ImuState / DriveStatus
├── vlm_perception/         VLM 客户端、感知节点、深度扇区、Mock 服务、假相机
├── vlm_control/            反应式规划器、A* 栅格规划（grid_map）、IMU 状态估计、真值里程计、控制节点
├── vlbot_description/      纯 URDF：差速底盘 + RGB 相机 + RGB-D 相机 + IMU + 激光雷达
├── vlbot_gazebo/           仓库世界（26x18m）/ 走廊世界 + 插件 + spawn
└── vlm_avoid_bringup/      顶层 launch + 参数（avoidance.yaml / tunable.json）+ rviz 配置
tests/integration_mock.py   无 Gazebo 的端到端行为测试
scripts/                    安装、体检、自检、启动、测试
docs/                       架构细节 / 安装排障 / 调参
```

---

## 4. 话题与参数

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | 入 | 前置 RGB，10 Hz |
| `/scan` | `sensor_msgs/LaserScan` | 入 | 360° 激光雷达，720 线，10 Hz |
| `/perception/scan_sectors` | `SectorScan` | 内部 | 雷达版左/中/右最近距离 |
| `/goal_pose` | `geometry_msgs/PoseStamped` | 入 | **目的地**（rviz 2D Goal Pose 或 `scripts/goal.sh`） |
| `/goal_cancel` | `std_msgs/Empty` | 入 | 取消目的地，回到自主巡航 |
| `/plan` | `nav_msgs/Path` | 内部 | A* 算出的路径（odom 系），控制器逐点跟随 |
| `/map` | `nav_msgs/OccupancyGrid` | 双向 | SLAM 实时地图（A* 直接消费）；`ground_truth` 模式下由规划器发布离线地图 |
| `/odom` | `nav_msgs/Odometry` | 入 | 轮式里程计（会漂，仅作参考/降级） |
| `/slam/odom` | `nav_msgs/Odometry` | 入 | **SLAM 修正后的位姿**，控制器和 A* 用它 |
| `/ground_truth/odom` | `nav_msgs/Odometry` | 内部 | Gazebo 真值位姿（世界系），由 `ground_truth_odom` 换算成 `/odom` |
| `/wheel_odom` | `nav_msgs/Odometry` | 调试 | 轮式里程计（只用来对比漂移，不参与控制） |
| `/camera/depth/image_raw` | `sensor_msgs/Image` (32FC1) | 入 | 可选；没有就 `enable_depth:=false` |
| `/camera_depth/depth/image_raw` | 同上 | 入 | **仿真里用的是这个**（Gazebo 插件按 `camera_name + "/depth/image_raw"` 拼话题），`sim_avoid.launch.py` 已自动覆盖 |
| `/imu/data` | `sensor_msgs/Imu` | 入 | 100 Hz |
| `/perception/vlm/result` | `VlmObstacle` | 内部 | `risk/direction/action/label/stale/latency_ms` |
| `/perception/vlm/annotated` | `sensor_msgs/Image` | 调试 | 把模型判断画在图上，`rqt_image_view` 直接看 |
| `/perception/sectors/result` | `SectorScan` | 内部 | 左/中/右最近距离 |
| `/state/imu` | `ImuState` | 内部 | `roll/pitch/yaw` |
| `/vlm_avoid/status` | `DriveStatus` | 调试 | 当前状态机、风险、净空、看门狗系数 |
| `/cmd_vel` | `geometry_msgs/Twist` | 出 | 20 Hz |

**日常调参只改一个文件**：[`tunable.json`](src/vlm_avoid_bringup/config/tunable.json)。
它按节点分段，覆盖了速度、阈值、VLM 时序、深度外参、IMU 等全部可调项，
改完重启 launch 生效（`--symlink-install` 下直接改 `src/` 里的文件即可）。
优先级：`avoidance.yaml` < `tunable.json` < launch 参数。

常用几个：

| 参数 | 默认 | 作用 |
|---|---|---|
| `vlm_rate_hz` | 1.5 | 每秒推理次数；实际上限 = 1/单帧耗时 |
| `max_side` / `num_predict` | 384 / 96 | 两个最影响推理耗时的旋钮 |
| `keep_alive` | 30m | 模型驻留显存，避免冷启动 |
| `cruise_linear` / `max_linear` | 0.35 / 0.45 | 巡航速度 / 硬上限 |
| `avoid_angular` | 0.85 | 避让角速度（转弯半径 ≈ v/ω ≈ 0.41 m） |
| `stop_distance` / `obstacle_distance` | 0.15 / 0.35 | **0.35 m 开始转向，0.15 m 硬停** |
| `release_margin` | 0.25 | 滞回：退到 0.60 m 才解除避让 |
| `clear_path_distance` | 0.55 | 前方够空时，侧向障碍只转向不减速 |
| `lateral_swerve_max_time` / `_angle` | 1.0 s / 0.4363 rad | 一次侧向避让最多转 **1 s 或 25°**，防止被模型误报带着绕圈 |
| `obstacle_risk_threshold` | 0.70 | 融合风险阈值；正前方障碍另有一票否决 |
| `hold_time` | 1.2 | 避让承诺窗口（防 VLM 抖动） |
| `vlm_stale_timeout` | 3.0 | 多久没结果算过期（自动按 3×延迟放宽） |
| `goal_tolerance` | 0.30 | 距目标多近算到达 (m) |
| `goal_kp` / `goal_max_angular` | 1.40 / 0.90 | 目的地导航的航向比例增益 / 角速度上限 |
| `goal_slow_angle` / `goal_slow_radius` | 0.90 / 1.20 | 角度大于它先原地转；距离小于它开始减速 |
| `tilt_estop` | 0.90 | IMU 倾角急停阈值 (rad) |
| `require_imu` | false | 真机建议 true：IMU 不在线不许动 |

完整参数表（含建议范围、公式、三套预设、热改命令）见 [docs/tuning.md](docs/tuning.md)。

运行期改参数不用重启：
```bash
ros2 param set /avoidance_controller cruise_linear 0.2
ros2 param set /vlm_obstacle_node rate_hz 2.0
```

---

## 5. 8GB 显存怎么选模型

| 模型 | 量化后显存 | 4060 单帧延迟 | 建议 |
|---|---|---|---|
| `qwen2.5vl:3b` | ~3.5 GB | 0.6~1.5 s | **默认**，留足显存给 Gazebo/桌面 |
| `qwen3-vl:4b` | ~4 GB | 0.8~2 s | 空间推理更强，允许 1 Hz 时优选 |
| `qwen2.5vl:7b` | ~6.5 GB | 1.5~3 s | 能跑，但和 Gazebo 抢显存容易 OOM |
| 任何模型 + CPU | — | 3~8 s | 驱动没修好时的兜底，链路能通但只适合"走走停停" |

想再快，两个旋钮：`max_side`（512→384）和 `num_predict`（128→96）。
换 vLLM 可把延迟压到 0.2~0.4 s，但 8 GB 下建议 3B-AWQ，见 [docs/setup.md](docs/setup.md)。

---

## 6. 降级矩阵（安全设计）

| VLM | 深度 | IMU | 行为 |
|---|---|---|---|
| 正常 | 正常 | 正常 | CRUISE / AVOID / GOTO，全功能 |
| 正常 | 雷达断 | 正常 | 只用深度，行为不变（日志里 scan 无效） |
| 正常 | 缺失 | 正常 | 纯语义避障，保留承诺窗口 + 航向保持 |
| 缺失 | 正常 | 正常 | 0.7 倍巡航，靠几何通道避障，`stale=True` |
| 缺失 | 缺失 | 正常 | 蠕行 6 s 后 STOP |
| 从未收到 | 缺失 | — | 直接 STOP（不盲开） |
| 任意 | 任意 | 倾角 > 0.9 rad | **ESTOP**，立即零速 |

`DriveStatus.state` 取值：`CRUISE / CAUTION / AVOID_LEFT / AVOID_RIGHT / STOP / RECOVERY / ESTOP`。
`STOP` 持续 4 s 会触发 `RECOVERY`（原地转向脱困），避免卡死。

---

## 7. 已验证 vs 待你验证

**已验证（本机实跑）**
- `colcon build`：6 个包全部通过
- 100 个测试：规划器状态机、JSON 容错解析、图像编解码、深度扇区与**地面过滤**、IMU 滤波、
  Mock HTTP 往返、launch 文件可加载、Gazebo 资产（`gz sdf` 转换与 world 校验）
- **真实 Qwen2.5-VL-3B（CPU 推理）+ 真实 ROS 管线**（关闭深度通道，避让只能来自 VLM）：
  - 正前方红箱 → 模型 `{"obstacle": true, "risk": 0.68, "direction": "center", "label": "红色立方体"}`，暖机 3.2 s
  - 控制器 → `AVOID_LEFT, risk=0.45, v=0.10 m/s, w=+0.60 rad/s` ✓
  - 合成场景（纯色几何体）下模型对"通畅"有误报：risk 0.45 → 只降到 `CAUTION 0.20 m/s`，不会急停；真实 Gazebo 纹理场景应更好
- 端到端（假相机 + Mock VLM + 真实 launch/DDS）：
  - 通畅 → `v=0.30 m/s`，`CRUISE`
  - 前方 0.9 m 障碍 → `AVOID_LEFT`，`max|ω|=0.75`，`max v=0.10`，深度扇区识别到 0.9 m
  - 视野恢复 → 回到 `CRUISE` 并继续前进
  - IMU 模拟倾覆 → `ESTOP`，指令严格归零
  - 杀掉 VLM 服务 → `stale=True`，限速 ≤0.21 m/s
  - 调试叠加图正常发布
  - 激光雷达扇区与深度扇区都识别出 0.30 m 障碍（两路逐项取小）
  - 目的地导航：给左侧 3 m 目标 → `GOTO` 且 `w=+0.90`；给脚下目标 → `ARRIVED` 且速度归零
  - **A\* 绕墙**：目标设在墙 A 正后方时，规划出 4 个路点的绕行路径（向北绕，最北 y=0.95m），
    控制器朝第一段路点转向（w=+0.17；直连目标时该值为 0）

**待你验证（需要桌面会话）**
- Gazebo 里相机/IMU 插件的**运行时**联调：
  `bash scripts/run_sim.sh` → 应看到 rviz 里小车前进、`/camera/image_raw` 有画面、
  `/odom` 有变化、`/cmd_vel` 随障碍变化。资产本身已用 `gz sdf` 静态校验通过。
- GPU 下的真实延迟：CPU 冷启动 14 s / 暖机 3.2 s，4060 上预计 0.5~1.5 s
  （跑 `bash scripts/run_sim.sh` 后看 `/perception/vlm/result` 里的 `latency_ms`）

**本机实际安装位置**（无需 sudo 的那份）

| 内容 | 路径 |
|---|---|
| Ollama 0.40.5 | `~/.local/bin/ollama` + `~/.local/lib/ollama/`（2.1 GB，sha256 已校验） |
| 模型 `qwen2.5vl:3b` | `~/.ollama/models`（3.0 GB，Ollama 默认目录） |
| 服务 | `127.0.0.1:11434`，任意终端 `ollama serve` 或用 `bash scripts/ollama_serve.sh` |
| PATH | `~/.local/bin`（已写入 `~/.bashrc`，新终端直接 `ollama ...`） |

---

## 8. 排障

### 8.1 "小车不动"怎么查（最常见的问题）

**先明确一点：这条链路不需要你输入任何指令**。感知正常时控制器会自己发 `/cmd_vel`；
不动通常是安全策略在起作用。按这个顺序查：

```bash
python3 scripts/doctor.py
```

它会直接给结论，对应关系如下：

| doctor 输出 | 含义 | 处理 |
|---|---|---|
| `感知完全没进来 → 安全停车` | 相机没图、深度没图、VLM 没结果 | 按设计停车。先 `ros2 topic hz /camera/image_raw`，没图就是 Gazebo 相机插件/渲染问题 |
| `控制器被 enable=false 关掉` | 有人（或 teleop 脚本）关了自动控制 | `python3 scripts/doctor.py --resume` |
| `IMU 倾角超限导致 ESTOP` | 车翻了 / IMU 姿态异常 | 看 `/state/imu` 的 roll/pitch |
| `控制器主动 STOP（净空不足/高风险）` | 感知认为前方不能走 | 看 `/perception/vlm/annotated` 和 `/vlm_avoid/status` |
| `有速度指令但里程没动` | 底盘没响应 | 看 Gazebo 是否 `pause:=true`、差速插件是否加载、spawn 高度 |
| `没有 /odom` | 只在跑 mock（假相机不发里程） | 决策链路正常，底盘未验证 |
| `✓ 一切正常` | 状态、指令、位移都对 | — |

还想单独确认底盘：`bash scripts/teleop.sh`（自动先关控制器，避免两个发布者打架）。

> 常见误判：以为要 `ros2 topic pub /cmd_vel` 才会动。实际上手动发指令会和控制器
> 20Hz 的输出**互相覆盖**，车会抖。要手动控制，先 `--disable`。



| 现象 | 原因 / 处理 |
|---|---|
| `nvidia-smi` 报错 | 驱动没装好：`ubuntu-drivers devices` → `sudo apt install nvidia-driver-XXX` → 重启；期间 Ollama 走 CPU |
| `X Error ... GLX ... X_GLXCreateContext` | 没有可用的 GL 设备：确认 `nvidia-smi` 正常，或用集显跑 Gazebo（`__NV_PRIME_RENDER_OFFLOAD=0`），服务器上装 `xvfb` 后用 `xvfb-run` |
| Gazebo 刷 `Read-only file system: ~/.gazebo` | HOME 不可写（容器/沙箱）：`HOME=/tmp/gz-home gzserver ...` |
| Gazebo 黑屏/崩溃 | 双显卡笔记本常见：`__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia gzclient`，或直接用集显跑 Gazebo（VLM 用独显） |
| 没显示器 | `xvfb-run -a bash scripts/run_sim.sh gui:=false`（相机渲染需要 GL 上下文） |
| VLM 一直 `stale` | `ollama list` 看模型在不在；`curl 127.0.0.1:11434/api/tags`；模型名必须和 `model` 参数一致 |
| 单帧 3 s 以上 | 在跑 CPU：确认 `nvidia-smi` 能看到 ollama 进程；或把 `max_side` 降到 384 |
| `VLM 调用失败（连续 N 次）: timeout` | 推理超时。CPU 推理单帧 10~16 s，把 `timeout` 调到 60~120；想真正实时就上 GPU（`bash scripts/ollama.sh ps` 看 PROCESSOR 是否 100% GPU） |
| rviz 里机器人模型不显示 | `/robot_description` 是**锁存**话题，订阅端必须 `Durability: Transient Local`（`avoid.rviz` 已设）。另检查 `ros2 topic echo /robot_description --once` 有 URDF、Global Status 里 Fixed Frame `map` 是否存在（rtabmap 模式要等它发布 map→odom） |
| RViz 和 Gazebo 对不上、路径像穿墙 | 位姿来源不对：仿真里 `/odom` 必须是**真值**（p3d 插件 → `ground_truth_odom`）；用轮式里程计撞墙后会漂。看 `ground_truth_odom` 日志里的"轮式里程计漂移 xx m"。另外确认 rviz 的 Fixed Frame 是 `odom` |
| 车在原地/大圈绕 | VLM 对侧墙误报 `direction=left` + 侧向避让无终止条件。已加"深度佐证 + 转角预算(1 s/25°) + 冷却"；仍绕就调 `lateral_swerve_max_angle` ↓ 或 `side_push_max` ↓ |
| 改了 `tunable.json` 没效果 | 键名拼错会被静默忽略。跑 `python3 -m pytest tests/test_tunable_json.py -q`，它专门检查键名是否存在 |
| 三个扇区距离都是 ~0.5 m、状态一直 STOP | 深度把**脚下地板**当成了障碍。本项目已按相机外参剔除地面点；真机换相机后要改 `camera_height` / `camera_pitch` / `horizontal_fov`（设 `camera_height:=0` 可关闭过滤） |
| `ModuleNotFoundError: No module named 'vlm_avoid_interfaces'` | 没 source 工作区。现在 `scripts/*.py` 会自动 source；若仍报错说明还没 `colcon build` |
| `/odom`、`/imu/data` 有数据但 `/camera/image_raw` 0 Hz | Gazebo 相机插件的话题名不是你以为的那个：ROS2 用 `camera_name`（snake_case）+ `camera_name/image_raw` 拼话题，写 ROS1 的 `cameraName` 会被静默忽略。跑 `python3 scripts/doctor.py` 看"图像话题"那一段的真实名字 |
| 机器人原地打转 | `avoid_angular` 太大或 `hold_time` 太长；先看 `/vlm_avoid/status` 的 `risk/clearance` |
| 不避让直接撞 | 画面里障碍物太小/太远：`stop_distance` 调大，或提高 `vlm_rate_hz`；先看 `/perception/vlm/annotated` 确认模型看到了 |

更多见 [docs/setup.md](docs/setup.md)、[docs/architecture.md](docs/architecture.md)、[docs/tuning.md](docs/tuning.md)。

---

## 9. 下一步可以怎么加

1. **动态障碍 + 重规划**：现在的 A* 用的是静态地图，新增的障碍靠本地避障躲；
   把激光雷达接进 costmap 后，可以在局部检测到新障碍时把栅格标脏并重新规划。
2. **VLA 端到端**：Qwen-VL 直接出离散动作 token，用本仓库的状态机做安全层（保底）。这层现在是现成的。
3. **真车**：`use_sim_time:=false require_imu:=true`，把 `/camera/image_raw`、`/imu/data` 换成真实驱动，`/cmd_vel` 接底盘；先低速 `max_linear:=0.15` 跑。
4. **SLAM 建图**：现在的地图是 `.world` 离线生成的（已知场景）；换到真机/未知场景时，
   用 `slam_toolbox` 在线建图，再把它的 `/map` 喂给现在的 A* 即可。

---

## 10. 定位与 SLAM：三次实验记录（含历史）

> 这一节按时间顺序记录"为什么这么选、怎么验证的"。旧实验不删——它们是排障和对比的基准。

### 10.0 起因

早期现象：**RViz 里的车和 Gazebo 里的车越走越偏，A* 路径像从墙里穿过去**。

排查结论：地图本身没错（`tests/test_map_generation.py` 验证了墙的坐标、目标标记不进图、
A* 绕行 26 m），错的是**位姿来源**——差速插件的轮式里程计靠轮速积分，撞墙、打滑后就漂。

### 10.1 实验一：Gazebo 真值位姿 + 离线地图（保留，仅用于对比/排障）

| | |
|---|---|
| 目的 | 先证明"地图 + A* + 控制器"这条几何链路是对的，把定位问题隔离出去 |
| 方法 | `gazebo_ros_p3d` 发世界真值 → `ground_truth_odom` 换算成"相对出生点"的 `/odom` + TF；地图由 `scripts/make_map_from_world.py` 从 `.world` 生成 |
| 结果 | RViz / Gazebo / A* 完全重合，绕墙路径正确 |
| 局限 | **真机没有真值**，只能当基准，不能当方案 |
| 现在怎么跑 | `bash scripts/run_sim.sh localization:=ground_truth` |

保留它的原因：任何"定位看起来不对"的问题，都可以切到这个模式跑一次——
如果真值模式下也对不上，那就是地图/坐标系的 bug；如果只有 SLAM 模式对不上，那是定位精度问题。

### 10.2 实验二：RTAB-Map（当前默认，真方案）

选型对比（ROS2 Humble，2024 前后主流）：

| 框架 | 传感器 | 回环/图优化 | ROS2 支持 | 结论 |
|---|---|---|---|---|
| **RTAB-Map** | RGB-D + 激光 + IMU | ✅ | 官方 apt 包，维护活跃 | **选它** |
| ORB-SLAM3 | 单目/双目/RGB-D（+IMU） | ✅ | 无官方 ROS2 包，编译重 | 纯视觉，不吃雷达，仿真纹理少时易跟丢 |
| VINS-Fusion | 相机 + IMU | ✅（VIO） | 需自行移植 | 只做里程计，不建 2D 栅格图 |
| slam_toolbox | 2D 激光 | ✅ | 官方 apt 包 | 很稳，但**不用相机/IMU**，不满足"三传感器" |
| Cartographer | 2D/3D 激光 | ✅ | ROS2 支持较弱 | 偏重，配置复杂 |

**RTAB-Map 的接法**：

```mermaid
flowchart LR
  RGB["/camera_depth/image_raw"] --> RT["RTAB-Map<br/>视觉/ICP 里程计 + 回环"]
  DEP["/camera_depth/depth/image_raw"] --> RT
  INFO["/camera_depth/camera_info"] --> RT
  SCAN["/scan"] --> RT
  IMU["/imu/data"] --> RT
  RT -->|"/map"| PLAN["A* 规划器"]
  RT -->|"TF map→odom→base"| BR["tf_pose_bridge"]
  BR -->|"/slam/odom"| CTRL["控制器 / A*"]
```

- 里程计可选：`odometry:=icp`（激光 ICP，仿真/空旷场景更稳，默认）或 `odometry:=visual`（RGB-D 视觉，纹理丰富时更好）
- 相机仍然参与：RGB-D 用于建图与**视觉回环检测**
- `tf_pose_bridge` 把 `map→base_footprint` 的 TF 转成项目统一的 `/slam/odom`，
  这样换框架时控制器和 A* **一行都不用改**

**验证状态**（实测，2026-10）：
- ✅ `ros-humble-rtabmap-ros 0.23.13` 已安装，`icp_odometry` / `rtabmap` / `tf_pose_bridge` 三节点正常拉起
- ✅ **用合成传感器数据跑通了整条 SLAM 链路**（`tests/rtabmap_smoke.py`）：
  `Odom: ratio=0.88` → `rtabmap (283): RTAB-Map=0.0155s` → `/map 402x464 @0.05m frame=map` → `/slam/odom` 有输出
- ⚠️ 用真实 Gazebo 相机时的建图效果仍需你确认（本开发环境没有 GPU 设备节点）

**踩过的坑：`/map` 一直是空的**
RTAB-Map 的节点运行在 `/rtabmap` 命名空间下，`map_topic` 参数默认是相对名 `map`
→ 实际发到 **`/rtabmap/map`**，而 A*/rviz 订阅的是 `/map`，于是"没有建图、没有路径、车不动"。
现在 launch 里用绝对名钉死：`map_topic:=/map`。自查命令：

```bash
ros2 topic info /map          # 应该有 1 个 publisher
ros2 topic hz /map            # 建图时约 1Hz
```

```bash
bash scripts/install_slam.sh          # 已装可跳过
bash scripts/run_sim.sh               # 默认 localization:=rtabmap
bash scripts/goal.sh 22 -6
```

### 10.3 实验三：仓库内置 SLAM（零依赖备用）

装不了包/想改算法时用它。组成：

1. **前端**：激光扫描对扫描匹配（似然场 + 由粗到精搜索），
   IMU 陀螺给 yaw 先验，轮式里程计给平移先验，RGB-D `ORB + PnP` 视觉里程计提供第三路估计；
2. **后端**：位姿图 + 回环检测（对历史关键帧做相对匹配）+ Gauss-Seidel 迭代优化；
3. **地图**：对数几率占用栅格（射线投射），直接发 `/map` 给 A*。

**验证方法**（不需要 Gazebo）：`scripts/slam_sim.py` 用 `warehouse_map.json` 射线投射出
"完美激光雷达"，让机器人沿闭合路线走，给轮速加 5% 尺度误差 + yaw 偏置，比较漂移和 SLAM 误差：

| 场景 | 轮式里程计漂移 | 内置 SLAM 误差 |
|---|---|---|
| 1 圈 18 m | 1.65 m | **0.30 m** |
| 2 圈 36 m | 3.28 m | **0.38 m** |

```bash
python3 scripts/slam_sim.py            # 复现上面的数
bash scripts/run_sim.sh localization:=slam   # 在真实管线里跑
```

### 10.4 三种模式怎么选

| 模式 | 位姿来源 | 地图 | 依赖 | 用途 |
|---|---|---|---|---|
| `localization:=rtabmap`（默认） | RGB-D + 激光 + IMU | SLAM 实时建 | rtabmap-ros | 真方案 |
| `localization:=slam` | 激光 + IMU + 轮速 + RGB-D | 内置 SLAM 实时建 | 无 | 零依赖备用 |
| `localization:=ground_truth` | Gazebo 真值 | 离线地图 | 无 | 对比/排障基准 |

> 真机上把 `localization:=ground_truth` 换掉即可，其余接口不变。
