#!/usr/bin/env bash
# 后台启动 Ollama 服务（幂等）。二进制和模型都在用户目录，与工作区无关。
#
#   bash scripts/ollama_serve.sh
set -eo pipefail

BIN="${OLLAMA_BIN:-$(command -v ollama 2>/dev/null || true)}"
[[ -x "$BIN" ]] || BIN="$HOME/.local/bin/ollama"
LOG="$HOME/.ollama/serve.log"
PIDFILE="$HOME/.ollama/serve.pid"

if [[ ! -x "$BIN" ]]; then
  echo "未找到 ollama，先执行: bash scripts/install_ollama_noroot.sh" >&2
  exit 1
fi
if curl -fsS http://127.0.0.1:11434/api/tags >/dev/null 2>&1; then
  echo "服务已在运行（11434）"
  exit 0
fi
mkdir -p "$HOME/.ollama"
nohup "$BIN" serve >"$LOG" 2>&1 &
echo $! > "$PIDFILE"
for _ in $(seq 1 30); do
  sleep 1
  if curl -fsS http://127.0.0.1:11434/api/tags >/dev/null 2>&1; then
    echo "✓ 已启动 (pid $(cat "$PIDFILE"))，日志 $LOG"
    echo
    echo "确认推理设备（PROCESSOR 列应为 100% GPU）："
    echo "  ollama run qwen2.5vl:3b '说一句你好'   # 先热身，加载模型"
    echo "  ollama ps"
    exit 0
  fi
done
echo "启动失败，看 $LOG" >&2
exit 1
