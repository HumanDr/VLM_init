#!/usr/bin/env bash
# 无需 root 的 Ollama 安装：装进用户目录，任意终端都能直接 `ollama`。
#
#   bash scripts/install_ollama_noroot.sh                 # 默认 qwen2.5vl:3b
#   bash scripts/install_ollama_noroot.sh qwen3-vl:4b
#   OLLAMA_DOWNLOAD_MIRROR= bash scripts/install_ollama_noroot.sh   # 强制直连 GitHub
#
# 安装位置（标准用户目录，不污染系统、不依赖本仓库）：
#   二进制  ~/.local/bin/ollama
#   运行库  ~/.local/lib/ollama/
#   模型    ~/.ollama/models/
#   日志    ~/.ollama/serve.log
#
# 已经用官方 install.sh 装过 system-wide 版的，用 scripts/setup_vlm.sh 即可，二者二选一。
set -eo pipefail

MODEL=${1:-qwen2.5vl:3b}
VER=${OLLAMA_VERSION:-v0.40.5}
PREFIX="$HOME/.local"                     # 解压后得到 ~/.local/bin + ~/.local/lib/ollama
MODELS_DIR="$HOME/.ollama/models"         # Ollama 的默认模型目录，无需设 OLLAMA_MODELS
DL_DIR="$HOME/.cache/ollama-download"
TARBALL="$DL_DIR/ollama-linux-amd64.tar.zst"
OFFICIAL="https://github.com/ollama/ollama/releases/download/${VER}/ollama-linux-amd64.tar.zst"
MIRROR_BASE=${OLLAMA_DOWNLOAD_MIRROR-https://gh-proxy.com/}
EXPECTED_SHA=${OLLAMA_SHA256:-dc5eabc292f9949546c90a97c820f924b035e05243b0de13c7f366182562169c}

mkdir -p "$PREFIX" "$MODELS_DIR" "$DL_DIR"

echo "== 1/6 获取二进制 =="
if [[ -x "$PREFIX/bin/ollama" ]]; then
  echo "  已存在：$("$PREFIX/bin/ollama" --version 2>&1 | tail -1)"
else
  if [[ -s "$TARBALL" ]] && [[ "$(sha256sum "$TARBALL" | awk '{print $1}')" == "$EXPECTED_SHA" ]]; then
    echo "  已有完整安装包，跳过下载"
  else
    rm -f "$TARBALL"
    # 直连 GitHub 在国内常只有 ~100KB/s，优先用加速镜像（下载后强制校验 sha256）
    for U in "${MIRROR_BASE}${OFFICIAL}" "$OFFICIAL"; do
      [[ -z "$MIRROR_BASE" && "$U" != "$OFFICIAL" ]] && continue
      echo "  下载: ${U:0:60}..."
      if curl -L --retry 3 --retry-delay 2 --connect-timeout 15 -o "$TARBALL" "$U" 2>/dev/null; then
        GOT=$(sha256sum "$TARBALL" | awk '{print $1}')
        [[ "$GOT" == "$EXPECTED_SHA" ]] && { echo "  ✓ sha256 校验通过"; break; }
        echo "  ! sha256 不匹配（$GOT），换下一个源"
        rm -f "$TARBALL"
      fi
    done
    [[ -s "$TARBALL" ]] || { echo "下载失败，可设置 OLLAMA_DOWNLOAD_MIRROR 或手动下载" >&2; exit 1; }
  fi
  echo "  解压到 $PREFIX（bin/ + lib/ollama/）"
  tar --zstd -xf "$TARBALL" -C "$PREFIX"
fi
"$PREFIX/bin/ollama" --version 2>&1 | tail -1

echo "== 2/6 配置 PATH（任意终端可用）=="
if [[ ":$PATH:" == *":$HOME/.local/bin:"* ]]; then
  echo "  当前 PATH 已包含 ~/.local/bin"
fi
if grep -q "user-local bin (ollama)" "$HOME/.bashrc" 2>/dev/null; then
  echo "  ~/.bashrc 已配置过，跳过"
else
  cp -a "$HOME/.bashrc" "$HOME/.bashrc.bak-ollama"
  cat >> "$HOME/.bashrc" <<'EOF'

# >>> user-local bin (ollama) >>>
# 让 ~/.local/bin 里的命令（ollama 等）在任意交互终端可用
case ":$PATH:" in
  *":$HOME/.local/bin:"*) ;;
  *) export PATH="$HOME/.local/bin:$PATH" ;;
esac
# <<< user-local bin (ollama) <<<
EOF
  echo "  已写入 ~/.bashrc（备份 ~/.bashrc.bak-ollama）；新开终端生效"
fi

echo "== 3/6 启动服务（模型目录 $MODELS_DIR）=="
if curl -fsS http://127.0.0.1:11434/api/tags >/dev/null 2>&1; then
  echo "  已有服务在 11434，直接复用"
else
  nohup "$PREFIX/bin/ollama" serve >"$HOME/.ollama/serve.log" 2>&1 &
  echo $! > "$HOME/.ollama/serve.pid"
  for _ in $(seq 1 30); do
    sleep 1
    curl -fsS http://127.0.0.1:11434/api/tags >/dev/null 2>&1 && break
  done
  curl -fsS http://127.0.0.1:11434/api/tags >/dev/null || {
    echo "服务起不来，看 ~/.ollama/serve.log" >&2; exit 1; }
  echo "  ✓ 已启动 (pid $(cat "$HOME/.ollama/serve.pid"))"
fi

echo "== 4/6 拉取模型 ${MODEL} =="
"$PREFIX/bin/ollama" pull "$MODEL"

echo "== 5/6 模型清单与推理设备 =="
"$PREFIX/bin/ollama" list
"$PREFIX/bin/ollama" ps 2>/dev/null || true

echo "== 6/6 真实视觉自检 =="
if [[ -f "$(dirname "${BASH_SOURCE[0]}")/vlm_selftest.py" ]]; then
  python3 "$(dirname "${BASH_SOURCE[0]}")/vlm_selftest.py" --model "$MODEL"
else
  echo "  （跳过：没找到 scripts/vlm_selftest.py）"
fi

cat <<EOF

完成（无需 sudo，任意终端可用）：
  二进制: ~/.local/bin/ollama
  运行库: ~/.local/lib/ollama/
  模型  : $MODELS_DIR
  服务  : ollama serve（或 bash scripts/ollama_serve.sh）

日常使用（新终端直接敲）：
  ollama list
  ollama run $MODEL
  ollama ps
EOF
