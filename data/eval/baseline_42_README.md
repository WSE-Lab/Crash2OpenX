# baseline_42 — the frozen benchmark set

**All downstream experiments (baseline, ablations, ADS evaluation, saturation, coverage claims) must be scoped to these 42 cases.** Do not silently expand or shrink this list — if the frozen set changes, bump the name (`baseline_42_v2`) and record the diff.

> Canonical numbers (v5, frozen 2026-07-02 — matches `baseline_42.json` and the paper):
> **713 reports → 490 expressible → 120 distinct signatures → K=42 → 393/490 = 80.2% coverage; 27/42 with prior CARLA runs reused.**
> Earlier revisions of this README cited 394/491 / 118 signatures / 11 prior runs — those were pre-v5 numbers and are superseded.

## What it is

- 42 case_ids drawn from `outputs/signature/` covering the top **80.2% (393/490)** of the clusterable scenario classes in the full 713-report DMV/NHTSA corpus
- Each case represents a *distinct signature bucket* (`(topology, lanes_bin, sut_maneuver, npc_sig)` quotient)
- Each case's `scene_seed` is `status="supported"` — full pipeline (XODR + XOSC + CARLA) is possible
- 27 / 42 already have prior CARLA runs in `outputs/medoid_runs/` (reusable)

## Files

- `baseline_42.json` — canonical case list (rank / case_id / bucket_size / cumulative_pct / signature), v5
- `baseline_42_exclude.txt` — the 12 case_ids replaced at freeze time, with per-case reasons
- `eval_medoids_top42.json` — full audit (bucket members, prior_carla flags, `distinct_signatures=120`)
- `eval_cluster_report_top42.md` — human-readable table + narrative preview per medoid

## Selection protocol (for reproducibility)

1. Extract 4-axis signatures from all 713 PDF reports: `tools/api_infer_signature.py` (schema `sig_v1`)
2. Keep the 490 *expressible* reports (both seeds return `status="supported"`); the other 223 exit via `unsupported` / `needs_extension`
3. Group by identical `(topology, lanes_bin, sut_maneuver, npc_sig)` → **120** distinct signatures
4. Rank buckets by size, tie-break by lex-first case_id
5. Take smallest K such that cumulative coverage ≥ 80% → **K = 42** (393/490 = 80.2%)
6. Per bucket, choose medoid case_id with priority: (a) `scene_seed=supported`, (b) prior CARLA run available, (c) lex-first
7. **Freeze pass (v5, 2026-07-02):** attempt full compile+run for every candidate medoid; 12 candidates failed and were replaced by the next member of their bucket (see `baseline_42_exclude.txt`): 4 vocabulary-gate refusals (`unsupported`/`needs_extension`), 4 WF-gate refusals (WF8 ×3, WF10 on case 671), 2 open `osc_blocks` lane-section compiler bugs, 2 infrastructure (CARLA server offline). The first 8 are the validation layers working as designed; disclosed in the paper as a selection threat to validity.
8. Regenerate anytime: `uv run python tools/select_top_medoids.py --coverage 0.80` (then re-apply the exclude list and re-freeze)

## What NOT to do

- Do not eyeball extra cases and add them to the baseline
- Do not drop the small-bucket medoids (#28-#42) because they are "just outliers" — they cover the long tail and their inclusion is what makes the 80% coverage claim honest
- Do not re-run the signature inference on a subset without regenerating the full baseline afterwards (bucket sizes shift)

## Downstream expectations

- **XOSC build** — 42/42 XSD-conformant XODR/XOSC pairs
- **CARLA execution** — 42/42 load & execute (verdicts: 1 ok / 4 drifted / 37 deadlock, see `outputs/medoid_runs/_summary.tsv`)
- **ADS evaluation** — 42 × N_ads runs
- **Reporting** — always cite `baseline_42` as the scope; use `393/490 = 80.2%` as the coverage claim
