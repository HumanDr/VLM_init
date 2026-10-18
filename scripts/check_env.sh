#!/usr/bin/env bash
# 一键体检：ROS / Gazebo / GPU / 渲染 / Ollama / 模型 / 工作区构建状态。
# 只读，不改系统。
set -o pipefail

ok()   { echo "  ✓ $*"; }
bad()  { echo "  ✗ $*"; }
warn() { echo "  ! $*"; }

echo "===== 1. 操作系统 / ROS ====="
grep -E '^PRETTY_NAME' /etc/os-release || true
if [[ -z "${ROS_DISTRO:-}" ]]; then
  [[ -f /opt/ros/humble/setup.bash ]] && source /opt/ros/humble/setup.bash
fi
if [[ -n "${ROS_DISTRO:-}" ]]; then ok "ROS_DISTRO=${ROS_DISTRO}"; else bad "未安装 ROS2 Humble"; fi

echo "===== 2. Gazebo Classic ====="
for p in gazebo_ros gazebo_plugins; do
  [[ -d "/opt/ros/${ROS_DISTRO:-humble}/share/$p" ]] && ok "$p" || bad "$p 缺失 → bash scripts/install_deps.sh"
done
if command -v gzserver >/dev/null 2>&1; then ok "$(gzserver --version 2>/dev/null | head -1)"; else bad "gzserver 缺失"; fi

echo "===== 3. GPU / 驱动 ====="
if command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi -L 2>/dev/null | grep -q GPU; then
  nvidia-smi --query-gpu=name,memory.total,memory.used,driver_version --format=csv,noheader | sed 's/^/    /'
  ok "NVIDIA 驱动可用"
else
  bad "nvidia-smi 不可用（驱动/CUDA 未就绪）→ apt install nvidia-driver-XXX 后重启"
  warn "此时 Ollama 会退到 CPU：3B 模型仍可跑通链路，但单帧延迟会到 3~8s"
fi
[[ -d /dev/dri ]] && ok "/dev/dri 存在（Gazebo 可用硬件渲染）" || warn "/dev/dri 不存在：Gazebo 只能用软件渲染，画面会慢"

echo "===== 4. 显示环境 ====="
if [[ -n "${DISPLAY:-}" ]]; then ok "DISPLAY=${DISPLAY}"; else
  warn "无 DISPLAY：用 gui:=false 或无头模式（Xvfb）"
  command -v xvfb-run >/dev/null && ok "xvfb-run 可用"
fi

echo "===== 5. Ollama / Qwen-VL ====="
OLLAMA_BIN="$(command -v ollama 2>/dev/null || true)"
[[ -x "$OLLAMA_BIN" ]] || OLLAMA_BIN="$HOME/.local/bin/ollama"
if [[ -x "$OLLAMA_BIN" ]]; then
  ok "$OLLAMA_BIN ($("$OLLAMA_BIN" --version 2>&1 | tail -1))"
  if [[ ":$PATH:" != *":$HOME/.local/bin:"* ]]; then
    warn "~/.local/bin 不在当前 PATH（新开终端生效；或执行 source ~/.bashrc）"
  fi
  if curl -fsS http://127.0.0.1:11434/api/tags >/dev/null 2>&1; then
    ok "服务在线 127.0.0.1:11434"
    echo "    已下载模型:"; "$OLLAMA_BIN" list 2>/dev/null | sed 's/^/      /'
    echo "    推理设备  :"; "$OLLAMA_BIN" ps 2>/dev/null | sed 's/^/      /'
  else
    warn "服务未启动 → bash scripts/ollama_serve.sh"
  fi
  [[ -d "$HOME/.ollama/models" ]] && ok "模型目录 ~/.ollama/models" || warn "~/.ollama/models 不存在"
else
  bad "未安装 ollama → bash scripts/install_ollama_noroot.sh（无需 sudo）"
fi

echo "===== 6. Python 依赖 ====="
python3 - <<'EOF'
mods = ['numpy', 'cv2', 'requests', 'rclpy', 'PIL']
missing = []
for m in mods:
    try:
        __import__(m)
        print(f'  ✓ {m}')
    except Exception:
        print(f'  ✗ {m}')
        missing.append(m)
raise SystemExit(0 if not missing else 1)
EOF

echo "===== 7. 工作区 ====="
if [[ -f "${PWD}/install/setup.bash" ]]; then ok "已构建 ${PWD}/install"; else bad "尚未构建 → colcon build --symlink-install"; fi

echo
echo "===== 结论 ====="
echo "必须补齐：Gazebo 包（若无）、Ollama 模型（若无）。"
echo "GPU 不可用时仍可先用 'ros2 launch vlm_avoid_bringup mock_demo.launch.py' 验证全链路。"
