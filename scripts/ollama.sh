#!/usr/bin/env bash
# 调用 Ollama（装在你自己的 ~/.local/bin 里，任意终端可用）。
#
#   bash scripts/ollama.sh list
#   bash scripts/ollama.sh run qwen2.5vl:3b
#   bash scripts/ollama.sh ps
#
# 模型目录用 Ollama 默认的 ~/.ollama/models，不依赖本仓库路径。
set -eo pipefail

BIN="${OLLAMA_BIN:-$(command -v ollama 2>/dev/null || true)}"
[[ -x "$BIN" ]] || BIN="$HOME/.local/bin/ollama"
if [[ ! -x "$BIN" ]]; then
  echo "未找到 ollama。先执行: bash scripts/install_ollama_noroot.sh" >&2
  echo "（或把 ~/.local/bin 加进 PATH 后重开终端）" >&2
  exit 1
fi
exec "$BIN" "$@"
