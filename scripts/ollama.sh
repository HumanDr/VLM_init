#!/usr/bin/env bash
# 使用工作区内的 Ollama（无需 sudo 安装的那份）。
#   bash scripts/ollama.sh list
#   bash scripts/ollama.sh run qwen2.5vl:3b
#   bash scripts/ollama.sh pull qwen2.5vl:7b
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
export OLLAMA_MODELS="$PWD/.tools/ollama-models"
BIN="$PWD/.tools/ollama/bin/ollama"
if [[ ! -x "$BIN" ]]; then
  echo "未找到 $BIN，先执行: bash scripts/install_ollama_noroot.sh" >&2
  exit 1
fi
exec "$BIN" "$@"
