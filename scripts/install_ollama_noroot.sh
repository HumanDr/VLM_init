#!/usr/bin/env bash
# 无需 root 的 Ollama 本地安装（装到工作区 .tools/ 下，不污染系统）。
#
#   bash scripts/install_ollama_noroot.sh                 # 默认 qwen2.5vl:3b
#   bash scripts/install_ollama_noroot.sh qwen3-vl:4b
#   OLLAMA_DOWNLOAD_MIRROR= bash scripts/install_ollama_noroot.sh   # 强制直连 GitHub
#
# 装了 system-wide 版（curl -fsSL https://ollama.com/install.sh | sh）的话不要用这个，
# 用 scripts/setup_vlm.sh 就行。两者二选一。
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

MODEL=${1:-qwen2.5vl:3b}
VER=${OLLAMA_VERSION:-v0.40.5}
PREFIX="$PWD/.tools/ollama"
MODELS_DIR="$PWD/.tools/ollama-models"
DL_DIR="$PWD/.tools/ollama-download"
TARBALL="$DL_DIR/ollama-linux-amd64.tar.zst"
OFFICIAL="https://github.com/ollama/ollama/releases/download/${VER}/ollama-linux-amd64.tar.zst"
MIRROR_BASE=${OLLAMA_DOWNLOAD_MIRROR-https://gh-proxy.com/}
EXPECTED_SHA=${OLLAMA_SHA256:-dc5eabc292f9949546c90a97c820f924b035e05243b0de13c7f366182562169c}

mkdir -p "$PREFIX" "$MODELS_DIR" "$DL_DIR"

echo "== 1/5 获取二进制 =="
if [[ -x "$PREFIX/bin/ollama" ]]; then
  echo "  已存在：$("$PREFIX/bin/ollama" --version 2>/dev/null | head -1)"
else
  if [[ -s "$TARBALL" ]] && [[ "$(sha256sum "$TARBALL" | awk '{print $1}')" == "$EXPECTED_SHA" ]]; then
    echo "  已有完整安装包，跳过下载"
  else
    rm -f "$TARBALL"
    # 直连 GitHub 在国内常常只有 100KB/s，优先用加速镜像（下载后强制校验 sha256）
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
  echo "  解压到 $PREFIX"
  tar --zstd -xf "$TARBALL" -C "$PREFIX"
fi
"$PREFIX/bin/ollama" --version | head -1

echo "== 2/5 启动服务（模型目录 $MODELS_DIR）=="
if curl -fsS http://127.0.0.1:11434/api/tags >/dev/null 2>&1; then
  echo "  已有服务在 11434 端口，直接复用"
else
  OLLAMA_MODELS="$MODELS_DIR" nohup "$PREFIX/bin/ollama" serve \
    >"$PWD/.tools/ollama-serve.log" 2>&1 &
  echo $! > "$PWD/.tools/ollama.pid"
  for _ in $(seq 1 30); do sleep 1; curl -fsS http://127.0.0.1:11434/api/tags >/dev/null 2>&1 && break; done
  curl -fsS http://127.0.0.1:11434/api/tags >/dev/null || {
    echo "服务起不来，看 .tools/ollama-serve.log" >&2; exit 1; }
  echo "  ✓ 服务已启动 (pid $(cat "$PWD/.tools/ollama.pid"))"
fi

echo "== 3/5 拉取模型 ${MODEL} =="
OLLAMA_MODELS="$MODELS_DIR" "$PREFIX/bin/ollama" pull "$MODEL"

echo "== 4/5 模型清单 =="
OLLAMA_MODELS="$MODELS_DIR" "$PREFIX/bin/ollama" list

echo "== 5/5 真实视觉自检 =="
python3 scripts/vlm_selftest.py --model "$MODEL"

cat <<EOF

完成（本地安装，无需 sudo）：
  二进制: $PREFIX/bin/ollama
  模型  : $MODELS_DIR
  服务  : OLLAMA_MODELS=$MODELS_DIR $PREFIX/bin/ollama serve

日常使用：
  bash scripts/ollama.sh list
  bash scripts/ollama.sh run $MODEL
  bash scripts/ollama_serve.sh          # 后台起服务
EOF
