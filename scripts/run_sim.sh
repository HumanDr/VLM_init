#!/usr/bin/env bash
# 启动 Gazebo 仿真 + 视觉避障管线。
#
#   bash scripts/run_sim.sh                      # 带 GUI
#   bash scripts/run_sim.sh gui:=false           # 无 GUI（纯算法/服务器）
#   bash scripts/run_sim.sh vlm_model:=qwen2.5vl:7b
#   xvfb-run -a bash scripts/run_sim.sh gui:=false   # 无显示器的相机渲染
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOG_DIR=${ROS_LOG_DIR:-$PWD/.ros/log}
export ROS_HOME=${ROS_HOME:-$PWD/.ros/home}

# 若 Ollama 没起来，给个明确提示（不阻塞启动，管线会走降级）
if ! curl -fsS http://127.0.0.1:11434/api/tags >/dev/null 2>&1; then
  echo "[警告] Ollama 未响应，VLM 会标记 stale 并降级。启动: ollama serve" >&2
fi

exec ros2 launch vlm_avoid_bringup sim_avoid.launch.py "$@"
