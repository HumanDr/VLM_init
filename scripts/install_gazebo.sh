#!/usr/bin/env bash
# 只安装 Gazebo Classic 11 + ROS 插件（需要 sudo 密码）。
#
#   bash scripts/install_gazebo.sh
#
# 说明：
#  * 你的 apt 源已经是 USTC 镜像，实测可以解析全部依赖；
#  * 新增 106 个包、下载约 121 MB、装完约 600 MB；
#  * 装完会自动做一次 gzserver 无头启动自检。
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

source /opt/ros/humble/setup.bash
SUDO=${SUDO:-sudo}

echo "== 1/4 检查是否已安装 =="
if [[ -d /opt/ros/humble/share/gazebo_ros ]] && command -v gzserver >/dev/null 2>&1; then
  echo "  已安装：$(gzserver --version 2>/dev/null | head -1)  跳过 apt"
else
  echo "== 2/4 apt 安装（需要 sudo 密码）=="
  $SUDO apt-get update
  $SUDO apt-get install -y \
    gazebo \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-plugins \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-teleop-twist-keyboard
fi

echo "== 3/4 验证包 =="
missing=0
for p in gazebo_ros gazebo_plugins; do
  if [[ -d "/opt/ros/humble/share/$p" ]]; then echo "  ✓ $p"; else echo "  ✗ $p"; missing=1; fi
done
command -v gzserver >/dev/null && echo "  ✓ gzserver" || { echo "  ✗ gzserver"; missing=1; }
if [[ $missing -ne 0 ]]; then
  echo "安装不完整，请检查 apt 源" >&2
  exit 1
fi

echo "== 4/4 无头启动自检（3 秒）=="
if [[ -n "${DISPLAY:-}" ]]; then
  timeout 12 gzserver --verbose "$(ros2 pkg prefix vlbot_gazebo 2>/dev/null || echo)/share/vlbot_gazebo/worlds/corridor.world" \
    >/tmp/gz_selftest.log 2>&1 || true
else
  echo "  无 DISPLAY，用 xvfb-run"
  if command -v xvfb-run >/dev/null; then
    timeout 15 xvfb-run -a gzserver --verbose \
      "$(ros2 pkg prefix vlbot_gazebo 2>/dev/null || echo)/share/vlbot_gazebo/worlds/corridor.world" \
      >/tmp/gz_selftest.log 2>&1 || true
  fi
fi
if grep -qiE "error|could not" /tmp/gz_selftest.log 2>/dev/null; then
  echo "  ! 自检日志里有错误，见 /tmp/gz_selftest.log（多半是显卡/GL 问题，不影响后续排查）"
else
  echo "  ✓ gzserver 能启动"
fi

cat <<'EOF'

完成。下一步：
  bash scripts/run_sim.sh            # 带 GUI
  bash scripts/run_sim.sh gui:=false # 无 GUI

如果 Gazebo 黑屏/崩溃（双显卡笔记本常见）：
  __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia gzclient
或让 Gazebo 走集显、只让 Ollama 用独显。
EOF
