#!/usr/bin/env bash
# 无 Gazebo / 无 GPU 的全链路演示：假相机 + Mock VLM + 真实控制回路。
# 适合先确认"感知-决策-执行"逻辑没问题，再上仿真和真机。
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOG_DIR=${ROS_LOG_DIR:-$PWD/.ros/log}
export ROS_HOME=${ROS_HOME:-$PWD/.ros/home}

echo "启动后，另开终端切场景："
echo '  ros2 topic pub -1 /test/scene std_msgs/String "{data: center}"'
echo '  ros2 topic echo /vlm_avoid/status'
echo '  ros2 topic echo /perception/vlm/result'
echo

exec ros2 launch vlm_avoid_bringup mock_demo.launch.py "$@"
