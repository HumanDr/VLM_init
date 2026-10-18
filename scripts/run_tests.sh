#!/usr/bin/env bash
# 跑全部测试：42 个单元/冒烟测试 + 1 个端到端集成测试。
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOG_DIR=${ROS_LOG_DIR:-$PWD/.ros/log}
export ROS_HOME=${ROS_HOME:-$PWD/.ros/home}
export PYTHONPATH="$PWD/src/vlm_perception:$PWD/src/vlm_control:${PYTHONPATH:-}"

echo "===== 单元测试 ====="
python3 -m pytest src/vlm_perception/test src/vlm_control/test tests -q

echo
echo "===== 端到端集成测试（真实进程 + 真实 DDS）====="
python3 tests/integration_mock.py
