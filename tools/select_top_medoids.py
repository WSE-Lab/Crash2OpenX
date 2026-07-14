#!/usr/bin/env python3
"""Frequency-based top-K medoid selection.

Purpose:
  k-medoids on a coarse Hamming metric produces a degenerate k=2 solution
  when the signature space has a heavy dominant class + long tail. This
  script bypasses k-medoids and instead groups cases by *identical
  signature* (topology, lanes-bin, sut_maneuver, npc_sig — collision_pair
  dropped because ID naming (v1/v2/av) is unstable across LLM runs).

  The K-th medoid is selected so cumulative coverage first reaches
  --coverage (default 0.80). For n=713, this typically yields K≈42.

Medoid pick per bucket:
  1. prefer a case_id that already has an outputs/medoid_runs/<cid>/ dir
     (so prior CARLA runs are re-usable);
  2. else lexicographically-first case_id in the bucket (deterministic).

Outputs:
  - paper/working/eval_medoids_top{K}.json   — machine-readable medoid list
  - paper/working/eval_cluster_report_top{K}.md — audit + narrative preview
"""
from __future__ import annotations

import argparse
import json
import re
from collections import defaultdict
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]


def _bin_lane(lanes) -> str:
    if isinstance(lanes, dict):
        fwd = lanes.get("forward")
    else:
        fwd = lanes
    if not isinstance(fwd, int):
        return "?"
    if fwd <= 1:
        return "1"
    if fwd == 2:
        return "2"
    return "3+"


def _load_narrative(cid: str, text_dir: Path) -> str:
    p = text_dir / f"{cid}.txt"
    if not p.is_file():
        return ""
    txt = p.read_text(encoding="utf-8")
    # Extract the narrative sentence(s) — heuristic: find lines containing
    # AV / Waymo / Cruise / Zoox / vehicle ... in a sentence-shaped context.
    lines = [ln.strip() for ln in txt.split("\n") if len(ln.strip()) > 60]
    for ln in lines:
        low = ln.lower()
        if any(kw in low for kw in ["was traveling", "was proceeding", "was making",
                                      "was stopped", "was in a collision", "operating in",
                                      "came to a stop", "rear-ended", "made contact",
                                      "attempted a", "approached the", "attempting to"]):
            return ln[:400]
    # fallback: first substantive line
    for ln in lines:
        if not ln.startswith(("OL 316", "SECTION", "REPORT OF", "Instructions:")):
            return ln[:400]
    return ""


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--eval-set", type=Path, default=REPO_ROOT / "paper/working/eval_set_full.json")
    ap.add_argument("--signature-dir", type=Path, default=REPO_ROOT / "outputs/signature")
    ap.add_argument("--road-dir", type=Path, default=REPO_ROOT / "outputs/road_seed")
    ap.add_argument("--text-dir", type=Path, default=REPO_ROOT / "outputs/extracted_text")
    ap.add_argument("--carla-runs-dir", type=Path, default=REPO_ROOT / "outputs/medoid_runs")
    ap.add_argument("--scene-dir", type=Path, default=REPO_ROOT / "outputs/scene_seed",
                    help="prefer bucket members whose scene_seed status is 'supported' "
                         "(so full pipeline can produce XOSC)")
    ap.add_argument("--exclude", nargs="*", default=[],
                    help="explicit case_ids to blacklist (e.g. known-broken XOSC build). "
                         "Bucket falls back to next-best member.")
    ap.add_argument("--exclude-file", type=Path, default=None,
                    help="path to a file containing one case_id per line to exclude (safer than --exclude for case_ids with parentheses)")
    ap.add_argument("--coverage", type=float, default=0.80,
                    help="cumulative coverage threshold; K = smallest k such that top-k signatures cover this fraction")
    ap.add_argument("--out-medoids", type=Path, default=None)
    ap.add_argument("--out-report", type=Path, default=None)
    args = ap.parse_args()

    es = json.loads(args.eval_set.read_text(encoding="utf-8"))
    prior_carla = set()
    if args.carla_runs_dir.is_dir():
        prior_carla = {p.name for p in args.carla_runs_dir.iterdir() if p.is_dir()}

    # bucket by signature
    buckets: dict[tuple, list[dict]] = defaultdict(list)
    skipped: list[tuple[str, str]] = []
    for c in es["cases"]:
        cid = c["case_id"]
        sig_p = args.signature_dir / f"{cid}.json"
        rd_p = args.road_dir / f"{cid}.json"
        if not sig_p.is_file() or not rd_p.is_file():
            skipped.append((cid, "missing sig or road_seed"))
            continue
        sig_doc = json.loads(sig_p.read_text(encoding="utf-8"))
        road_doc = json.loads(rd_p.read_text(encoding="utf-8"))
        if sig_doc.get("status") != "supported" or road_doc.get("status") != "supported":
            skipped.append((cid, f"status: sig={sig_doc.get('status')} road={road_doc.get('status')}"))
            continue
        sig = sig_doc.get("signature") or {}
        topo = (road_doc.get("road") or {}).get("topology", "?")
        lb = _bin_lane((road_doc.get("road") or {}).get("lanes"))
        key = (topo, lb, sig.get("sut_maneuver"), tuple(sig.get("npc_sig") or []))
        buckets[key].append({"case_id": cid, "sig": sig, "road": road_doc.get("road")})

    total_clusterable = sum(len(v) for v in buckets.values())
    target = int(round(total_clusterable * args.coverage))

    # order buckets by size desc, tie-break by earliest case_id for determinism
    ordered = sorted(buckets.items(),
                     key=lambda kv: (-len(kv[1]), sorted(m["case_id"] for m in kv[1])[0]))

    exclude_set = set(args.exclude or [])
    if args.exclude_file and args.exclude_file.is_file():
        for line in args.exclude_file.read_text(encoding="utf-8").splitlines():
            s = line.strip()
            if s and not s.startswith("#"):
                exclude_set.add(s)
    medoids = []
    csum = 0
    for rank, (key, members) in enumerate(ordered, 1):
        # Filter out blacklisted case_ids
        eligible = [m for m in members if m["case_id"] not in exclude_set]
        if not eligible:
            # All members blacklisted — skip this bucket entirely.
            # (Common when the signature itself is WF-impossible, e.g.
            # 'adjacent' NPC on a 1-lane road violates WF8 for every member.)
            continue

        def score(cid: str) -> tuple:
            sp = args.scene_dir / f"{cid}.json"
            has_scene = False
            if sp.is_file():
                try:
                    has_scene = json.loads(sp.read_text(encoding="utf-8")).get("status") == "supported"
                except (OSError, json.JSONDecodeError):
                    pass
            in_carla = cid in prior_carla
            return (has_scene, in_carla, cid)

        # Sort: has_scene DESC, in_carla DESC, cid ASC (lex-first among tied top candidates)
        cids_by_score = sorted((m["case_id"] for m in eligible),
                               key=lambda c: (-int(score(c)[0]), -int(score(c)[1]), c))
        picked_cid = cids_by_score[0]
        picked = next(m for m in eligible if m["case_id"] == picked_cid)
        sp = args.scene_dir / f"{picked_cid}.json"
        picked_scene_status = "missing"
        if sp.is_file():
            try:
                picked_scene_status = json.loads(sp.read_text(encoding="utf-8")).get("status", "?")
            except (OSError, json.JSONDecodeError):
                pass
        reuse = picked_cid in prior_carla
        csum += len(members)
        pct = csum / total_clusterable
        topo, lb, mv, npc = key
        medoids.append({
            "rank": rank,
            "case_id": picked["case_id"],
            "prior_carla_run": reuse,
            "scene_seed_status": picked_scene_status,
            "bucket_size": len(members),
            "cumulative_covered": csum,
            "cumulative_pct": round(pct, 4),
            "signature": {
                "topology": topo,
                "lanes_bin": lb,
                "sut_maneuver": mv,
                "npc_sig": list(npc),
            },
            "members": sorted(m["case_id"] for m in members),
        })
        if csum >= target:
            break

    K = len(medoids)
    out_medoids = args.out_medoids or (REPO_ROOT / f"paper/working/eval_medoids_top{K}.json")
    out_report = args.out_report or (REPO_ROOT / f"paper/working/eval_cluster_report_top{K}.md")

    payload = {
        "version": "top_medoids_v1",
        "source_eval_set": str(args.eval_set.relative_to(REPO_ROOT)),
        "coverage_target": args.coverage,
        "clusterable_total": total_clusterable,
        "distinct_signatures": len(buckets),
        "K": K,
        "cumulative_covered": csum,
        "cumulative_pct": round(csum / total_clusterable, 4),
        "prior_carla_reused": sum(1 for m in medoids if m["prior_carla_run"]),
        "medoids": medoids,
        "skipped_count": len(skipped),
    }
    out_medoids.parent.mkdir(parents=True, exist_ok=True)
    out_medoids.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    lines = [f"# Top-K medoid set (frequency-based, coverage ≥ {args.coverage:.0%})", "",
             f"- eval set: `{args.eval_set.name}`",
             f"- clusterable: **{total_clusterable}** cases",
             f"- distinct signatures: **{len(buckets)}**",
             f"- **K = {K}** medoids cover **{csum}/{total_clusterable} = {csum/total_clusterable:.1%}** of clusterable cases",
             f"- prior CARLA runs reused: **{payload['prior_carla_reused']}/{K}**",
             f"- skipped (non-supported / missing seed): {len(skipped)}",
             "",
             "## Medoid list (rank by bucket size)",
             "",
             f"| # | size | cum% | prior CARLA | topology | lanes | maneuver | npc_sig | medoid case_id |",
             f"|---|---|---|---|---|---|---|---|---|"]
    for m in medoids:
        s = m["signature"]
        npc = ", ".join(s["npc_sig"]) or "-"
        prior = "✓" if m["prior_carla_run"] else ""
        lines.append(
            f"| {m['rank']} | {m['bucket_size']} | {m['cumulative_pct']:.1%} | {prior} | "
            f"{s['topology']} | {s['lanes_bin']} | {s['sut_maneuver']} | {npc} | `{m['case_id']}` |"
        )
    lines += ["", "## Narrative preview per medoid", ""]
    for m in medoids:
        s = m["signature"]
        narr = _load_narrative(m["case_id"], args.text_dir)
        prior_tag = " (prior CARLA)" if m["prior_carla_run"] else ""
        lines.append(f"### #{m['rank']} — `{m['case_id']}`{prior_tag}")
        lines.append(f"- signature: `{s['topology']} / {s['lanes_bin']} lanes / {s['sut_maneuver']} / npc={s['npc_sig']}`")
        lines.append(f"- represents **{m['bucket_size']}** case(s), cumulative coverage {m['cumulative_pct']:.1%}")
        lines.append(f"- narrative: {narr[:300] if narr else '(no narrative found in extracted text)'}")
        lines.append("")

    out_report.parent.mkdir(parents=True, exist_ok=True)
    out_report.write_text("\n".join(lines) + "\n", encoding="utf-8")

    print(f"K = {K}   coverage = {csum}/{total_clusterable} ({csum/total_clusterable:.1%})   prior CARLA reused = {payload['prior_carla_reused']}/{K}")
    print(f"wrote {out_medoids.relative_to(REPO_ROOT)}")
    print(f"wrote {out_report.relative_to(REPO_ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
