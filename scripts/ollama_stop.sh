#!/usr/bin/env bash
# 停止 Ollama 服务（释放显存）。
#
#   bash scripts/ollama_stop.sh
#
# 会依次尝试：
#   1) ~/.ollama/serve.pid 里记的进程
#   2) pkill -f "ollama serve"（覆盖你在任意终端手动起的那份）
#   3) 15 秒还没退就 SIGKILL
set -eo pipefail

URL=${OLLAMA_URL:-http://127.0.0.1:11434}
PIDFILE="$HOME/.ollama/serve.pid"

running() { curl -fsS "$URL/api/tags" >/dev/null 2>&1; }

if ! running; then
  echo "[ollama] 当前没有在运行（$URL 无响应）"
  rm -f "$PIDFILE"
  exit 0
fi

echo "[ollama] 正在停止 ..."

# 1) 优先优雅停止 pid 文件里的进程
if [[ -f "$PIDFILE" ]]; then
  pid="$(cat "$PIDFILE" 2>/dev/null || true)"
  if [[ -n "${pid:-}" ]] && kill -0 "$pid" 2>/dev/null; then
    kill "$pid" 2>/dev/null || true
    echo "[ollama] 已向 pid $pid 发送 SIGTERM"
  fi
fi

# 2) 兜底：任何 "ollama serve" 进程
pkill -f "ollama serve" 2>/dev/null || true

done_ok=0
for _ in $(seq 1 15); do
  sleep 1
  if ! running; then
    done_ok=1
    break
  fi
done

# 3) 还不退就强杀
if [[ "$done_ok" -ne 1 ]]; then
  echo "[ollama] 优雅停止超时，强制结束"
  pkill -9 -f "ollama serve" 2>/dev/null || true
  sleep 2
fi

rm -f "$PIDFILE"

if running; then
  echo "[ollama] 仍在监听 $URL。可能不是当前用户启动的，试试：" >&2
  echo "         ss -ltnp | grep 11434     # 看是哪个进程" >&2
  echo "         systemctl --user stop ollama   # 如果你装过 systemd 服务" >&2
  exit 1
fi

echo "[ollama] 已停止，显存已释放"
