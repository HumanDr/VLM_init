#!/usr/bin/env bash
# 后台启动工作区内的 Ollama 服务（幂等）。
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
export OLLAMA_MODELS="$PWD/.tools/ollama-models"
BIN="$PWD/.tools/ollama/bin/ollama"
LOG="$PWD/.tools/ollama-serve.log"

if [[ ! -x "$BIN" ]]; then
  echo "未找到 $BIN，先执行: bash scripts/install_ollama_noroot.sh" >&2
  exit 1
fi
if curl -fsS http://127.0.0.1:11434/api/tags >/dev/null 2>&1; then
  echo "服务已在运行"
  exit 0
fi
nohup "$BIN" serve >"$LOG" 2>&1 &
echo $! > "$PWD/.tools/ollama.pid"
for _ in $(seq 1 30); do
  sleep 1
  if curl -fsS http://127.0.0.1:11434/api/tags >/dev/null 2>&1; then
    echo "✓ 已启动 (pid $(cat "$PWD/.tools/ollama.pid"))，日志 $LOG"
    echo
    echo "确认推理设备（PROCESSOR 列显示 100% GPU / 100% CPU）："
    echo "  bash scripts/ollama.sh run qwen2.5vl:3b '说一句你好'   # 先热身，加载模型"
    echo "  bash scripts/ollama.sh ps"
    exit 0
  fi
done
echo "启动失败，看 $LOG" >&2
exit 1
