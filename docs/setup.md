# 安装与排障

## 1. 系统依赖

```bash
bash scripts/install_gazebo.sh      # 只要 Gazebo（推荐，带启动自检）
bash scripts/install_deps.sh        # Gazebo + xacro + rviz + rqt + 调试工具
```

实测（本机，2026-10）：`apt-get install -s` 解析出 **106 个新包**，
下载约 **121 MB**，源是 USTC 镜像（`https://mirrors.ustc.edu.cn/ros2/ubuntu`）。
如果卡住，先换源：

```bash
sudo sed -i 's|packages.ros.org/ros2/ubuntu|mirrors.ustc.edu.cn/ros2/ubuntu|g' \
  /etc/apt/sources.list.d/ros2-latest.list
sudo sed -i 's|archive.ubuntu.com|mirrors.ustc.edu.cn|g; s|security.ubuntu.com|mirrors.ustc.edu.cn|g' \
  /etc/apt/sources.list
sudo apt update
```

等价于：

```bash
sudo apt update
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins \
  ros-humble-xacro ros-humble-joint-state-publisher ros-humble-robot-state-publisher \
  ros-humble-rviz2 ros-humble-rqt-image-view ros-humble-teleop-twist-keyboard \
  python3-numpy python3-opencv python3-requests python3-pytest python3-pip
```

装完验证：

```bash
gzserver --version
ros2 pkg list | grep gazebo
```

## 2. NVIDIA 驱动（4060 + 8GB）

```bash
nvidia-smi                        # 有输出才说明驱动 OK
ubuntu-drivers devices            # 看推荐版本
sudo apt install nvidia-driver-XXX
sudo reboot
```

笔记本双显卡注意：Ubuntu 默认会用到 NVIDIA（PRIME on-demand 时不会）。
如果 `nvidia-smi` 正常但 Gazebo 卡顿/黑屏，可让 Gazebo 跑集显、VLM 跑独显：

```bash
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia gzclient
```

驱动没修好也能干活：Ollama 会自动走 CPU，链路能通但单帧 3~8 s，
建议把 `rate_hz` 降到 1.0、`cruise_linear` 降到 0.15。

## 3. Ollama + Qwen-VL

```bash
bash scripts/setup_vlm.sh                  # 默认 qwen2.5vl:3b
bash scripts/setup_vlm.sh qwen3-vl:4b      # 想换更新一代
bash scripts/setup_vlm.sh qwen2.5vl:7b     # 8GB 勉强（~6.5GB），别同时开 Gazebo
```

### 没有 sudo 密码？装到用户目录

```bash
bash scripts/install_ollama_noroot.sh   # 装进 ~/.local，模型进 ~/.ollama/models
ollama list                             # 新终端直接可用（PATH 已写入 ~/.bashrc）
bash scripts/ollama_serve.sh            # 后台起服务（幂等）
```

安装位置都是**标准用户目录**，和本仓库无关：

| 内容 | 路径 |
|---|---|
| 二进制 | `~/.local/bin/ollama` |
| 运行库 | `~/.local/lib/ollama/` |
| 模型 | `~/.ollama/models/`（Ollama 默认目录，不需要设 `OLLAMA_MODELS`） |
| 日志 / pid | `~/.ollama/serve.log`、`~/.ollama/serve.pid` |

原理：下载 `ollama-linux-amd64.tar.zst` → 解压到 `~/.local` → `bin/ollama` 会自己
找到 `../lib/ollama` 下的运行库。脚本会用官方 `sha256sum.txt` **强制校验**
（当前版本 v0.40.5，sha256 `dc5eabc2…2169c`），并幂等地把 `~/.local/bin`
写进 `~/.bashrc` 的 PATH。

### Ollama 需要可写的 HOME

`ollama serve` 会在 `~/.ollama/` 下生成私钥（`id_ed25519`）和配置。
如果 `$HOME` 只读（容器、沙箱、CI），会报：

```
Error: could not create directory mkdir /home/xxx/.ollama: read-only file system
```

解决：把 HOME 指到可写目录（模型目录单独用 `OLLAMA_MODELS` 指定）：

```bash
mkdir -p /tmp/ollama-home
HOME=/tmp/ollama-home OLLAMA_MODELS=/tmp/ollama-models \
  ~/.local/bin/ollama serve
```

正常桌面机器不需要这一步。

### 确认 Ollama 用的是 GPU

```bash
bash scripts/ollama.sh run qwen2.5vl:3b '你好'   # 热身，把模型加载进显存
bash scripts/ollama.sh ps
# NAME           ID  SIZE     PROCESSOR    UNTIL
# qwen2.5vl:3b   ..  3.4 GB   100% GPU     4 minutes from now
```

- `100% GPU` → 单帧约 0.5~1.5 s，`vlm_rate_hz: 1.0` 够用；
- `100% CPU` → 单帧 10~16 s，把 `timeout` 设到 60~120、`vlm_rate_hz` 降到 0.2；
  根因通常是 `nvidia-smi` 不通（驱动）或 Ollama 装了但没带 CUDA 库。

#### 8 GB 显存怎么分（实测 RTX 4060 Laptop）

| 占用 | 实测/估计 |
|---|---|
| Ollama + qwen2.5vl:3b (Q4_K_M) | **~4.0 GB**（`ollama ps` 显示 2.8 GB 模型、3.96 GB 显存） |
| Xorg + GNOME | ~0.3 GB |
| Gazebo 640x480 渲染 | ~1.0~1.5 GB |
| rviz2 | ~0.3~0.6 GB |
| 浏览器等其它 | 0.2~0.5 GB |

合计 ~6 GB，余量约 2 GB。显存紧张时的降级顺序：

1. `bash scripts/run_sim.sh rviz:=false`（省 0.5 GB）
2. 把 URDF 里 RGB 相机降到 320x240 / `update_rate` 降到 5
3. `vlm_obstacle_node` 的 `max_side` 512 → 384（画面更小，KV cache 更省）
4. 关掉浏览器

另外 Ollama 默认 **5 分钟无请求就卸载模型**。隔了很久再启动仿真，第一帧会有 5~10 s
的加载时间（`latency_ms` 偏高），之后恢复到 0.5~1.5 s，属正常。

注意：**服务必须在你自己的终端里启动**，才能看到 GPU 设备。
如果 11434 已被别的实例占用（例如容器里的 CPU 版），先 `pkill -f "ollama serve"` 再：

```bash
bash scripts/ollama_serve.sh
```

### 国内下载：官方直连很慢

| 源 | 实测速度 | 说明 |
|---|---|---|
| `github.com/.../ollama-linux-amd64.tar.zst` | ~95 KB/s | 1.44 GB 要 4 小时，基本不可用 |
| `gh-proxy.com/<github url>` | ~3 MB/s | 脚本默认用它，下载后校验 sha256 |
| `ollama.com/download/ollama-linux-amd64.tgz` | **404** | 新版已改名 `.tar.zst`，官方旧链接失效 |
| `registry.ollama.ai`（模型 blob） | ~1.7 MB/s | 3.2 GB 约 30 分钟 |

可覆盖：`OLLAMA_DOWNLOAD_MIRROR= bash scripts/install_ollama_noroot.sh`（强制直连）
或 `OLLAMA_DOWNLOAD_MIRROR=https://ghfast.top/ ...`。

手动等价操作：

```bash
curl -fsSL https://ollama.com/install.sh | sh
ollama serve &                             # 或 systemctl --user start ollama
ollama pull qwen2.5vl:3b
ollama list
python3 scripts/vlm_selftest.py --model qwen2.5vl:3b
```

自检脚本会打印：模型原始返回、解析后的 JSON、单帧延迟。
若它说"没把红箱子判为障碍"，先别怀疑 ROS 管线，换模型或调提示词。

**加速手段**（按收益排序）：

1. `max_side` 512→384（像素少约 45%，延迟线性下降）
2. `num_predict` 128→96（输出更短）
3. `rate_hz` 1.0（别指望 2 Hz 以上，实时性靠几何通道）
4. 换 vLLM（见下）

## 4. 备选：vLLM + Qwen2.5-VL-3B-AWQ

```bash
pip install vllm
vllm serve Qwen/Qwen2.5-VL-3B-Instruct-AWQ \
  --port 8000 --max-model-len 4096 --gpu-memory-utilization 0.55
```

然后：

```bash
ros2 launch vlm_avoid_bringup sim_avoid.launch.py \
  api_style:=openai vlm_url:=http://127.0.0.1:8000 vlm_model:=Qwen/Qwen2.5-VL-3B-Instruct-AWQ
```

8 GB 显存下 `--gpu-memory-utilization` 别超 0.6，否则和 Gazebo 抢显存。

## 5. 无显示器的仿真（服务器 / SSH）

Gazebo 的相机传感器需要一个 GL 上下文，即使 `gui:=false` 也可能崩：

```bash
sudo apt install -y xvfb
xvfb-run -a -s "-screen 0 1280x1024x24" bash scripts/run_sim.sh gui:=false rviz:=false
```

软件渲染慢，可以把相机降到 320×240（改 URDF 里 `width/height` 和 `update_rate`）。

## 5.5 Gazebo 相机话题的两个坑

1. **标签是 snake_case**：ROS2 的 `gazebo_ros_camera` 只认 `<camera_name>` / `<frame_name>`。
   写成 ROS1 的 `<cameraName>` / `<frameName>` 不会报错，而是被**静默忽略**，
   于是 `camera_name` 默认取传感器名，话题变成 `/camera/front_camera/image_raw`——
   `/camera/image_raw` 永远没数据。
2. **话题是 `camera_name + "/image_raw"`**，depth 传感器还会再多一层：
   `camera_name + "/depth/image_raw"`。所以本仓库用：

   | 传感器 | camera_name | 彩色 | 深度 | 点云 |
   |---|---|---|---|---|
   | RGB | `camera` | `/camera/image_raw` | — | — |
   | depth | `camera_depth` | `/camera_depth/image_raw`（不用） | `/camera_depth/depth/image_raw` | `/camera_depth/points` |

   仿真的 `sim_avoid.launch.py` 会把 `depth_topic` 自动指到后一个名字；
   真机上按你的驱动改 `image_topic` / `depth_topic` 即可。

排查时先看真实话题名，而不是猜：

```bash
ros2 topic list | grep -iE "image|points"
python3 scripts/doctor.py        # 会列出所有 Image 话题及 publisher 数
```

## 6. 常见错误对照

| 报错 / 现象 | 处理 |
|---|---|
| `Failed to load plugin libgazebo_ros_diff_drive.so` | 没装 `ros-humble-gazebo-plugins` |
| `spawn_entity.py: timeout` | `robot_description` 没发布：确认 description.launch.py 已启动；或改用 `-file` 方式 spawn |
| `[ERROR] [camera]: Unable to create GL context` | 无 DISPLAY 或驱动异常：用 `xvfb-run` 或修驱动 |
| `rclpy._rclpy_pybind11.RCLError: Failed to create log directory` | `$HOME` 不可写（容器/沙箱）：`export ROS_LOG_DIR=$PWD/.ros/log` |
| `ros2 topic list` 有话题但 `echo` 无数据 | 用 `ros2 topic hz /cmd_vel --no-daemon` 排查；确认 `ROS_DOMAIN_ID` 一致 |
| VLM 返回一堆中文解释而不是 JSON | 模型太弱/提示词被改坏；`format=json` 已在客户端强制，仍有解释就说明模型不支持该参数 |
| `CUDA out of memory` | 模型太大或 vLLM 占满显存；换 3B 或降 `--gpu-memory-utilization` |

## 7. 从仿真切到真车

1. `use_sim_time:=false`，`require_imu:=true`
2. 相机驱动把图像发到 `/camera/image_raw`（`bgr8` 或 `rgb8` 都支持，也支持 `mono8`）
3. IMU 发 `/imu/data`，确认单位是 `rad/s` 和 `m/s²`
4. 底盘订阅 `/cmd_vel`；**先架空轮子**验证转向符号：正 `angular.z` 应让车左转
5. 首轮用 `max_linear:=0.15 cruise_linear:=0.12`，确认 `ESTOP` 时底盘真的停
