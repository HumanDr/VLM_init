#!/usr/bin/env bash
# 安装仿真与调试所需的系统依赖（需要 sudo）。
# 已安装的包会自动跳过，可重复执行。
set -eo pipefail

SUDO=${SUDO:-sudo}

echo "== 检查 ROS 环境 =="
if [[ -z "${ROS_DISTRO:-}" ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi
echo "ROS_DISTRO=${ROS_DISTRO:-unknown}"

APT_PKGS=(
  # Gazebo Classic 11 + ROS 插件（相机/IMU/差速驱动/spawn）
  ros-humble-gazebo-ros-pkgs
  ros-humble-gazebo-plugins
  # 机器人描述 / 可视化 / 调试
  ros-humble-xacro
  ros-humble-joint-state-publisher
  ros-humble-robot-state-publisher
  ros-humble-rviz2
  ros-humble-rqt-image-view
  ros-humble-teleop-twist-keyboard
  # Python 依赖
  python3-numpy
  python3-opencv
  python3-requests
  python3-pytest
  python3-pip
)

echo "== 安装 ${#APT_PKGS[@]} 个 apt 包（需要 sudo 密码）=="
$SUDO apt-get update
$SUDO apt-get install -y "${APT_PKGS[@]}"

echo "== 验证 =="
missing=0
for p in gazebo_ros gazebo_plugins robot_state_publisher rviz2; do
  if [[ -d "/opt/ros/humble/share/$p" ]]; then
    echo "  ✓ $p"
  else
    echo "  ✗ $p 缺失"
    missing=1
  fi
done
command -v gzserver >/dev/null && echo "  ✓ gzserver $(gzserver --version 2>/dev/null | head -1)" || { echo "  ✗ gzserver 缺失"; missing=1; }

if [[ $missing -ne 0 ]]; then
  echo "有依赖缺失，请检查 apt 源（建议使用国内镜像：mirrors.ustc.edu.cn/ros2）" >&2
  exit 1
fi
echo "== 完成。接着执行：bash scripts/setup_vlm.sh =="
