#!/usr/bin/env bash
# 手动遥控（用来确认底盘本身能动能转）。
#
#   bash scripts/teleop.sh
#
# 会自动先把避障控制器关掉（enable=false），否则 20Hz 的控制器会和键盘指令打架。
# 退出后再恢复：python3 scripts/doctor.py --resume
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

source /opt/ros/humble/setup.bash 2>/dev/null || true
[[ -f install/setup.bash ]] && source install/setup.bash

echo "== 关闭避障控制器（避免和遥控指令打架）=="
python3 scripts/doctor.py --disable || echo "  ! 没能关掉控制器，可能它没在跑"

echo
if ros2 pkg prefix teleop_twist_keyboard >/dev/null 2>&1; then
  echo "== 键盘遥控 =="
  echo "  前后/左右/turbo，q 退出。退出后恢复自动避障：python3 scripts/doctor.py --resume"
  echo
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
else
  cat <<'EOF'
没装 teleop_twist_keyboard（sudo apt install ros-humble-teleop-twist-keyboard）。
可以直接用下面几条命令验证底盘：

  # 前进 3 秒
  timeout 3 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"

  # 原地左转 3 秒（正角速度 = 左转）
  timeout 3 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"

  # 停车
  ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{}"

确认完恢复自动避障：python3 scripts/doctor.py --resume
EOF
fi
