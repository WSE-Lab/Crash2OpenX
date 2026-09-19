#!/usr/bin/env bash
# 保真度判官（R3-1）：生成的场景和事故报告说的是不是同一件事。
#
#   ./scripts/run_fidelity.sh probe                 # 哪些模型能当判官
#   ./scripts/run_fidelity.sh smoke "A,B"           # 2 个案例试跑
#   ./scripts/run_fidelity.sh full  "A,B"           # 全量 42，可断点续跑
#
# 判官至少给两个（逗号分隔），因为 v1 的判官就是产种子的那个模型，等于自己给
# 自己打分。两个判官才能报一致率。
#
# API key 从 .env.local 的 OPENROUTER_API_KEY 读，脚本自己会加载。
# 连不上就先 export http_proxy=http://127.0.0.1:7890 https_proxy=同上 再重试。
set -euo pipefail

# httpx 一看到 all_proxy 是 socks5 就要求装 socksio，否则直接 ImportError。
# 这里去掉它，只留 http_proxy/https_proxy——Clash 那个端口本身也收 HTTP 代理。
unset all_proxy ALL_PROXY
case "${http_proxy:-}${https_proxy:-}" in
  socks*) echo "[错误] http_proxy/https_proxy 是 socks 形式，httpx 走不了。"
          echo "       改成 HTTP 形式再跑：export http_proxy=http://127.0.0.1:7890 https_proxy=http://127.0.0.1:7890"
          exit 2 ;;
esac

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PY="$ROOT/.venv/bin/python"
JUDGE="$ROOT/tools/medoid_fidelity_v2.py"
OUT="$ROOT/outputs/fidelity_v2"

# probe 的候选。2026-08-05 实测结果（本账号，OpenRouter）：
#   看图 OK   qwen/qwen3-vl-235b-a22b-instruct、qwen/qwen3-vl-32b-instruct、x-ai/grok-4.3
#   空响应    xiaomi/mimo-v2.5、z-ai/glm-4.6v、moonshotai/kimi-k2.6
#   403       google/*、anthropic/*、openai/*（provider 政策，账号层面就被拦）
#   无图片端点 deepseek/deepseek-v4-pro（只能当纯文本判官）
CANDIDATES="qwen/qwen3-vl-235b-a22b-instruct,x-ai/grok-4.3,qwen/qwen3-vl-32b-instruct,deepseek/deepseek-v4-pro"

preflight() {
  local reports seeds frames
  reports=$(ls "$ROOT"/data/eval/baseline_direct/reports/*.txt 2>/dev/null | wc -l | tr -d ' ')
  seeds=$(ls "$ROOT"/data/eval/fidelity_42/scene_seed/*.json 2>/dev/null | wc -l | tr -d ' ')
  frames=$(ls -d "$ROOT"/outputs/medoid_runs/*/keyframes 2>/dev/null | wc -l | tr -d ' ')
  echo "报告 $reports/42   种子 $seeds/42   有关键帧的案例 $frames"
  grep -q '^OPENROUTER_API_KEY=.' "$ROOT/.env.local" || { echo "[错误] .env.local 里没有 OPENROUTER_API_KEY"; exit 2; }
  [ "$reports" = 42 ] && [ "$seeds" = 42 ] || { echo "[错误] 输入不齐，先补齐再跑"; exit 2; }
}

case "${1:-}" in
  probe)
    preflight
    echo "--- 纯文本 ---"
    "$PY" "$JUDGE" --probe --judges "$CANDIDATES"
    echo "--- 带关键帧（要判仿真画面就得看这一轮） ---"
    "$PY" "$JUDGE" --probe --vlm --judges "$CANDIDATES"
    echo
    echo "两轮都 OK 的模型才能进 smoke/full。EMPTY RESPONSE 和 FAIL 都不能用。"
    ;;

  smoke)
    [ $# -ge 2 ] || { echo "用法: $0 smoke \"judgeA,judgeB\""; exit 2; }
    preflight
    "$PY" "$JUDGE" --judges "$2" --vlm --limit 2
    echo
    echo "先看一个案例的原始输出，确认每条要素都带了报告原文引用："
    echo "  cat $OUT/*/\$(ls $OUT/*/ | head -1)"
    ;;

  full)
    [ $# -ge 2 ] || { echo "用法: $0 full \"judgeA,judgeB\""; exit 2; }
    preflight
    # --resume：已judge过的案例直接读缓存，中途断了重跑不重复烧 token
    "$PY" "$JUDGE" --judges "$2" --vlm --resume --workers 4
    echo
    echo "结果：$OUT/results.md（分数、失效分布、判官一致率、信号灯案例分层）"
    ;;

  *)
    sed -n '2,12p' "${BASH_SOURCE[0]}"
    exit 2
    ;;
esac
