#!/usr/bin/env bash
# 安装并拉起 Ollama，拉取 Qwen-VL 模型，并做一次真实的视觉自检。
#
#   bash scripts/setup_vlm.sh                 # 默认 qwen2.5vl:3b（8GB 显存友好）
#   bash scripts/setup_vlm.sh qwen2.5vl:7b    # 显存够的话
#   bash scripts/setup_vlm.sh qwen3-vl:4b     # 更新一代，空间推理更好
set -eo pipefail

MODEL=${1:-qwen2.5vl:3b}
if command -v ollama >/dev/null 2>&1; then
  echo "检测到已安装 Ollama: $(command -v ollama)"
  echo "如果只是想拉模型/起服务，直接用它即可，不必再装 system-wide 版："
  echo "  bash scripts/ollama_serve.sh && ollama pull ${MODEL}"
  read -r -p "仍要继续安装 system-wide 版？[y/N] " ans
  [[ "${ans:-N}" =~ ^[Yy]$ ]] || exit 0
fi
OLLAMA_URL=${OLLAMA_URL:-http://127.0.0.1:11434}

echo "== 1/4 检查 Ollama =="
if ! command -v ollama >/dev/null 2>&1; then
  echo "未检测到 ollama，开始安装（需要 sudo，会写入 /usr/local/bin 并创建 systemd 服务）"
  curl -fsSL https://ollama.com/install.sh | sh
else
  echo "  ✓ $(command -v ollama) ($(ollama --version 2>/dev/null | head -1))"
fi

echo "== 2/4 确保服务在跑 =="
if ! curl -fsS "${OLLAMA_URL}/api/tags" >/dev/null 2>&1; then
  echo "服务未响应，尝试后台启动 ollama serve"
  nohup ollama serve >"${HOME}/ollama.log" 2>&1 &
  for _ in $(seq 1 20); do
    sleep 1
    curl -fsS "${OLLAMA_URL}/api/tags" >/dev/null 2>&1 && break
  done
fi
curl -fsS "${OLLAMA_URL}/api/tags" >/dev/null || {
  echo "Ollama 仍不可达，请手动执行: ollama serve" >&2; exit 1; }
echo "  ✓ ${OLLAMA_URL} 可用"

echo "== 3/4 拉取模型 ${MODEL}（首次约 2~5GB，取决于量化）=="
ollama pull "${MODEL}"

echo "== 4/4 视觉自检（真实调用一次，量延迟）=="
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
python3 "${SCRIPT_DIR}/vlm_selftest.py" --url "${OLLAMA_URL}" --model "${MODEL}"

cat <<EOF

完成。后续启动管线时带上模型名：
  ros2 launch vlm_avoid_bringup sim_avoid.launch.py vlm_model:=${MODEL}
或直接改 config/avoidance.yaml 里的 model。

显存参考（INT4 量化）：
  qwen2.5vl:3b   ~3.5GB   4060 上单帧约 0.6~1.5s   ← 默认
  qwen3-vl:4b    ~4GB     空间推理更好
  qwen2.5vl:7b   ~6.5GB   8GB 可跑但余量小，易与 Gazebo 抢显存
EOF
