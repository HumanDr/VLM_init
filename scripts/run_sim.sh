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

# 残留的 Gazebo 进程会让新的一次启动卡住（spawn 服务/端口被旧进程占着）
if pgrep -f "gzserver|gzclient" >/dev/null 2>&1; then
  echo "[警告] 检测到残留的 Gazebo 进程，可能导致新仿真迟迟起不来：" >&2
  echo "         pkill -f gzserver; pkill -f gzclient" >&2
fi

# Ollama 由你自己手动管理：
#   启动: bash scripts/ollama_serve.sh
#   停止: bash scripts/ollama_stop.sh
# 这里只做一次提示，不阻塞仿真（VLM 会标记 stale，靠深度/雷达继续跑）。
if ! curl -fsS http://127.0.0.1:11434/api/tags >/dev/null 2>&1; then
  echo "[提示] Ollama 未响应，VLM 会降级。启动: bash scripts/ollama_serve.sh" >&2
fi

exec ros2 launch vlm_avoid_bringup sim_avoid.launch.py "$@"
