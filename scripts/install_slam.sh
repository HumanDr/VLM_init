#!/usr/bin/env bash
# 安装 SLAM 框架（需要 sudo）。装完把 localization:=rtabmap 打开即可。
#
#   bash scripts/install_slam.sh
#
# 装的是：
#   ros-humble-rtabmap-ros   RGB-D + 激光 + IMU 的图优化 SLAM（主力）
#   ros-humble-slam-toolbox  2D 激光 SLAM（备用，轻量）
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

if [[ "$(id -u)" -ne 0 ]]; then
  echo "需要 sudo，正在提权 ..."
  exec sudo -E bash "$0" "$@"
fi

apt-get update
apt-get install -y ros-humble-rtabmap-ros ros-humble-slam-toolbox

echo
echo "✓ 安装完成。验证："
echo "    ls /opt/ros/humble/share | grep -E 'rtabmap|slam_toolbox'"
echo "  启动（RGB-D + 雷达 + IMU 融合建图）："
echo "    bash scripts/run_sim.sh localization:=rtabmap"
