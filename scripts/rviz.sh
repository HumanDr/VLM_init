#!/usr/bin/env bash
# 单独启动 rviz2（不想等整条仿真管线时用）。
#
#   bash scripts/rviz.sh
#
# 等价于 ros2 run rviz2 rviz2 -d <share>/config/avoid.rviz
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
source /opt/ros/humble/setup.bash 2>/dev/null || true
[[ -f install/setup.bash ]] && source install/setup.bash
export ROS_LOG_DIR=${ROS_LOG_DIR:-$PWD/.ros/log}
export ROS_HOME=${ROS_HOME:-$PWD/.ros/home}
CFG=$(ros2 pkg prefix vlm_avoid_bringup)/share/vlm_avoid_bringup/config/avoid.rviz
echo "rviz2 -d $CFG"
exec rviz2 -d "$CFG"
