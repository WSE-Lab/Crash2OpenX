#!/bin/bash
# Full-corpus signature → cluster → saturation pipeline.
# Designed to run unattended overnight. Writes a single summary file at the end.
set -uo pipefail
cd "$(cd "$(dirname "$0")/.." && pwd)"

export http_proxy="http://127.0.0.1:7890"
export https_proxy="http://127.0.0.1:7890"

LOGDIR=/tmp/full_run_$(date +%Y%m%d_%H%M%S)
mkdir -p "$LOGDIR"
REPORT=paper/working/FULL_RUN_REPORT.md
T0=$(date +%s)

echo "=== STAGE 1: signature batch over 713 PDFs ==="
echo "log: $LOGDIR/batch.log"
uv run --python 3.12 --with openai --with pymupdf --with "markitdown[pdf]" \
    python tools/batch_infer_signatures.py --workers 5 \
    > "$LOGDIR/batch.log" 2>&1
BATCH_RC=$?
echo "stage 1 exit=$BATCH_RC"

echo "=== STAGE 2: cluster on full corpus ==="
uv run --python 3.12 python tools/cluster_eval_set.py \
    --eval-set paper/working/eval_set_full.json \
    --signature-dir outputs/signature \
    --out-medoids paper/working/eval_medoids_full.json \
    --out-report paper/working/eval_cluster_report_full.md \
    > "$LOGDIR/cluster.log" 2>&1
CLUSTER_RC=$?
echo "stage 2 exit=$CLUSTER_RC"

echo "=== STAGE 3: saturation curve ==="
uv run --python 3.12 python tools/cluster_saturation.py \
    --eval-set paper/working/eval_set_full.json \
    --signature-dir outputs/signature \
    --out paper/figures/cluster_saturation_full.pdf \
    > "$LOGDIR/saturation.log" 2>&1
SAT_RC=$?
echo "stage 3 exit=$SAT_RC"

T1=$(date +%s)
ELAPSED=$((T1 - T0))

# Aggregate status histogram
uv run --python 3.12 python - <<EOF
import json, glob
from collections import Counter
stats = Counter()
for p in glob.glob("outputs/signature/*.json"):
    if p.endswith(".failed.json"):
        continue
    try:
        d = json.load(open(p))
        stats[d.get("status", "?")] += 1
    except Exception:
        stats["read_error"] += 1
fail = len(glob.glob("outputs/signature/*.failed.json"))
print(f"status_hist={dict(stats)} failed_files={fail}")
EOF
> "$LOGDIR/status_hist.txt" 2>&1

# Pull key numbers out for the report
BATCH_TAIL=$(tail -20 "$LOGDIR/batch.log")
CLUSTER_TAIL=$(tail -30 "$LOGDIR/cluster.log")
SAT_TAIL=$(tail -20 "$LOGDIR/saturation.log")
STATUS_LINE=$(cat "$LOGDIR/status_hist.txt")

cat > "$REPORT" <<EOF
# Full-corpus run report

Pipeline: \`tools/batch_infer_signatures.py\` → \`cluster_eval_set.py --signature-dir\` → \`cluster_saturation.py --signature-dir\`

- Started: $(date -r $T0)
- Finished: $(date -r $T1)
- Elapsed: $((ELAPSED / 60))m $((ELAPSED % 60))s
- Exit codes: stage1=$BATCH_RC  stage2=$CLUSTER_RC  stage3=$SAT_RC
- Logs: \`$LOGDIR/\`

## Signature status histogram (all 713 cases)

\`\`\`
$STATUS_LINE
\`\`\`

## Stage 1 — signature batch (tail)

\`\`\`
$BATCH_TAIL
\`\`\`

## Stage 2 — cluster (tail)

\`\`\`
$CLUSTER_TAIL
\`\`\`

Outputs:
- medoid set: \`paper/working/eval_medoids_full.json\`
- audit report: \`paper/working/eval_cluster_report_full.md\`

## Stage 3 — saturation (tail)

\`\`\`
$SAT_TAIL
\`\`\`

Output: \`paper/figures/cluster_saturation_full.pdf\` (+ .png)

## Cost note

Roughly \$0.0058/case × ~613 newly-inferred cases ≈ \$3.5 on OpenRouter (dsv4-pro).
Exact figure: see openrouter dashboard.

## Next-day checklist

1. Open \`paper/working/eval_cluster_report_full.md\` — confirm new medoid set is reasonable.
2. Open \`paper/figures/cluster_saturation_full.png\` — confirm curve plateaus before n=713.
3. Decide whether to re-run CARLA on the **new** medoids (subset of \$\$\\Delta\$ medoid set\$\$).
4. If status_hist looks wrong (>5% read_error or fail), inspect \`$LOGDIR/batch.log\`.

EOF

echo "=== DONE in $((ELAPSED / 60))m, report at $REPORT ==="
