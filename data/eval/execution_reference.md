# 42-pattern execution reference (release verification)

Verification snapshot for the frozen 42-medoid benchmark, run from this
repository against a remote CARLA 0.9.16 host (RTX A6000) with the
InterFuser PCLA agent (`if_if`), 2026-07-15. Each pattern was executed twice
(60 s cap, then 90 s cap after the static-lead takeover-speed fix).

| Check | Result |
|---|---|
| Seeds present + OCL/collision intent declared | 42 / 42 |
| XODR + XOSC XSD-valid | 42 / 42 |
| Executed in CARLA (ticks, trajectories, evidence bundle) | 42 / 42 |
| Declared collision realized in ≥1 run (`aligned`) | 4 / 42 — cases 113, 122, 140, 249 |
| Remaining outcome | behavior drift: the scenario compiles and runs, but the ADS avoids the declared impact (typical signature: ego brakes to a stop 7–15 m behind the lead) |

Notes:

- Collision realization is **stochastic** in the ADS: case 122 realized the
  collision in one run and avoided it in another; non-medoid case 193
  realized it in 2/2 runs. Numbers above are per-snapshot, not upper bounds.
- Behavior drift is a *finding*, not a failure: the execution gate
  (`tools/scene_outcome.py`) exists precisely to separate "compilable and
  runnable" from "behaviorally aligned with the declared intent"
  (see the paper's L3 gate). Raw evidence per case (video, `events.jsonl`,
  `behavior_check.json`) lands under `outputs/medoid_runs/<case>/`.
- The demo case (165, freeway-ramp rear-end) realizes its declared collision
  through the full inference pipeline (temporal normalization maps the
  narrative to `front_brake`, hero boots at 12 m/s); the frozen seed in this
  benchmark predates that normalization and declares `stopped_ahead`.

Reproduce:

```bash
uv run python tools/batch_run_medoids.py --pcla-agent if_if --max-seconds 90
```
