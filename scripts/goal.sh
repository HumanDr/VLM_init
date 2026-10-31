#!/usr/bin/env bash
# 给机器人一个目的地。
#
#   bash scripts/goal.sh 22 -6        # 去仓库东南角的目标点
#   bash scripts/goal.sh 10 2         # 任意坐标（odom 坐标系，单位 m）
#   bash scripts/goal.sh cancel       # 取消目的地，恢复自主巡航
#
# 也可以在 rviz2 里用工具栏的 "2D Goal Pose" 点一下地面，效果一样。
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

source /opt/ros/humble/setup.bash 2>/dev/null || true
[[ -f install/setup.bash ]] && source install/setup.bash
export ROS_LOG_DIR=${ROS_LOG_DIR:-$PWD/.ros/log}
export ROS_HOME=${ROS_HOME:-$PWD/.ros/home}
mkdir -p "$ROS_LOG_DIR" "$ROS_HOME"

if [[ "${1:-}" == "cancel" ]]; then
  ros2 topic pub --once /goal_cancel std_msgs/msg/Empty "{}"
  echo "已取消目的地"
  exit 0
fi

if [[ $# -lt 2 ]]; then
  cat <<'EOF'
用法:
  bash scripts/goal.sh <x> <y>     # 例如: bash scripts/goal.sh 22 -6
  bash scripts/goal.sh cancel      # 取消

仓库场景推荐目标: (22, -6) —— 需要绕过两道隔墙，全程约 30 m
EOF
  exit 1
fi

X=$1
Y=$2
echo "发送目的地: x=$X y=$Y (frame=odom)"
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: odom}, pose: {position: {x: $X, y: $Y, z: 0.0}, orientation: {w: 1.0}}}"
