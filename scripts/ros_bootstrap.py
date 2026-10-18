"""让脚本在"没 source 环境"时也能直接跑。

自己在 ROS 环境里跑没问题；但常见情况是用户新开一个终端，只 source 了
/opt/ros/humble 甚至什么都没 source，于是 `python3 scripts/probe.py` 报
`ModuleNotFoundError: No module named 'vlm_avoid_interfaces'`。

这里检测工作区的 Python 包能不能导入；不能就找到 `<repo>/install/setup.bash`，
用 bash 重新 source 后 exec 回同一个解释器、同一份参数。
"""
from __future__ import annotations

import os
import shlex
import sys

_GUARD = '_VLN_ROS_BOOTSTRAPPED'


def _can_import() -> bool:
    try:
        import vlm_avoid_interfaces.msg  # noqa: F401
        import rclpy  # noqa: F401
        return True
    except Exception:  # noqa: BLE001
        return False


def ensure_env(verbose: bool = True) -> None:
    """必要时 source ROS + 工作区后重新执行当前脚本；成功则直接返回。"""
    if _can_import():
        return
    if os.environ.get(_GUARD) == '1':
        # 已经尝试过引导仍然失败：给出可操作的报错，而不是再绕一圈
        sys.stderr.write(
            '找不到 vlm_avoid_interfaces。请先构建并 source：\n'
            '  bash scripts/build.sh\n'
            '  source /opt/ros/humble/setup.bash && source install/setup.bash\n')
        raise SystemExit(2)

    repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    workspace_setup = os.path.join(repo, 'install', 'setup.bash')
    if not os.path.isfile(workspace_setup):
        sys.stderr.write(
            f'未找到 {workspace_setup}\n先构建工作区：bash scripts/build.sh\n')
        raise SystemExit(2)

    if verbose:
        print(f'[bootstrap] 自动 source {workspace_setup}', file=sys.stderr)

    log_dir = os.environ.get('ROS_LOG_DIR') or os.path.join(repo, '.ros', 'log')
    home_dir = os.environ.get('ROS_HOME') or os.path.join(repo, '.ros', 'home')
    os.makedirs(log_dir, exist_ok=True)
    os.makedirs(home_dir, exist_ok=True)

    script = os.path.abspath(sys.argv[0])
    args = ' '.join(shlex.quote(a) for a in sys.argv[1:])
    cmd = (
        'source /opt/ros/humble/setup.bash && '
        f'source {shlex.quote(workspace_setup)} && '
        f'export {_GUARD}=1 && '
        f'export ROS_LOG_DIR={shlex.quote(log_dir)} ROS_HOME={shlex.quote(home_dir)} && '
        f'exec {shlex.quote(sys.executable)} {shlex.quote(script)} {args}'
    )
    os.execvp('bash', ['bash', '-c', cmd])
