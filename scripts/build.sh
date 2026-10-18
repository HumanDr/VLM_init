#!/usr/bin/env bash
# 构建工作区（编译 6 个包）。
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

source /opt/ros/humble/setup.bash
export ROS_LOG_DIR=${ROS_LOG_DIR:-$PWD/.ros/log}
export ROS_HOME=${ROS_HOME:-$PWD/.ros/home}
mkdir -p "$ROS_LOG_DIR" "$ROS_HOME"

colcon build --symlink-install "$@"
echo
echo "构建完成。每次新开终端记得 source："
echo "  source /opt/ros/humble/setup.bash && source $PWD/install/setup.bash"
