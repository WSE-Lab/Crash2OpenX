#!/usr/bin/env python3
"""Cluster the stratified eval set on DSL features and pick medoids.

Pipeline:
  1. Read paper/eval_set.json (the deterministic stratified n=50 list).
  2. Join each case_id with road_seed/<id>.json + scene_seed/<id>.json.
  3. Keep only cases where BOTH seeds exist AND BOTH have status == "supported"
     (i.e. the pipeline can actually compile something downstream — a case with
     scene=supported but road=unsupported has no geometry and would surface as
     topology="?"/lanes="?" features, polluting the distance matrix).
  4. Construct a categorical feature vector per case (8 dims):
        (road.topology, road.lanes (binned), sut.maneuver, |npcs|,
         sorted set of "<npc.position>|<npc.behavior.block>",
         "ego↔<other>" collision-pair signature with role anonymised,
         environment.weather, environment.time_of_day)
  5. Distance = Hamming over the categorical features (lanes/|npcs| binned).
  6. Agglomerative (Ward-equivalent over precomputed distances → average linkage)
     with k swept; pick the k that maximises silhouette score over k ∈ [2, n-1].
  7. For each cluster, pick the medoid: the member with minimum sum of distances
     to its cluster-mates. Ties broken by case_id (deterministic).
  8. Write paper/eval_medoids.json + an audit report listing each cluster's
     members + selected medoid + feature summary so the choice is reviewable.

Determinism: SciPy's linkage uses a stable tie-break; medoid uses lexicographic
tie-break. No randomness anywhere — re-running yields the same medoids.
"""
from __future__ import annotations

import argparse
import json
from collections import defaultdict
from pathlib import Path
from typing import Any

import numpy as np
from scipy.cluster.hierarchy import fcluster, linkage
from scipy.spatial.distance import squareform

ROOT = Path(__file__).resolve().parents[1]


def _bin_lane(n) -> str:
    # Schema v2: lanes is {"forward": int, "backward": int}; legacy seeds may
    # still carry a bare int. Bin on `forward` (ego-direction count) so the
    # cluster labels remain comparable to the v1 "1"/"2" buckets — asymmetric
    # roads (forward=1, backward=2) now resolve to forward's value.
    if n is None:
        return "?"
    if isinstance(n, dict):
        n = n.get("forward")
        if n is None:
            return "?"
    if not isinstance(n, int):
        return "?"
    if n <= 1:
        return "1"
    if n == 2:
        return "2"
    return "3+"


def _bin_npc_count(n: int) -> str:
    if n == 0:
        return "0"
    if n == 1:
        return "1"
    return "2+"


def _feature_vector_from_signature(road: dict, sig_doc: dict) -> tuple:
    """sig_v1 path: scene 4 axes come from signature_v1, road 2 axes from road_seed.

    Returns the same 6-tuple shape as _feature_vector(), so downstream distance /
    silhouette / medoid logic is identical regardless of source.
    """
    r = road.get("road") or {}
    sig = sig_doc.get("signature") or {}
    npc_sig = tuple(sorted(set(sig.get("npc_sig") or [])))
    coll_pair = sig.get("collision_pair") or "-"
    return (
        r.get("topology", "?"),
        _bin_lane(r.get("lanes")),
        sig.get("sut_maneuver", "?"),
        _bin_npc_count(int(sig.get("npc_count") or 0)),
        npc_sig,
        coll_pair,
    )


def _feature_vector(road: dict, scene: dict) -> tuple:
    r = road.get("road") or {}
    sc = scene.get("scene") or {}
    sut = sc.get("sut") or {}
    npcs = sc.get("npcs") or []
    npc_sig = tuple(sorted({
        f"{n.get('position','?')}|{(n.get('behavior') or {}).get('block','?')}"
        for n in npcs
    }))
    coll = sc.get("collision") or {}
    coll_pair = "-"
    if coll.get("a") and coll.get("b"):
        # Normalise role: ego is always one side; we record the other party kind only.
        a, b = coll["a"], coll["b"]
        other = b if a == "ego" else (a if b == "ego" else f"{a}+{b}")
        coll_pair = f"ego↔{other}" if "ego" in (a, b) else f"{a}↔{b}"
    # Schema v2.2 freeze (commit fbe7497) introduced scene.environment but treats
    # weather + time_of_day as **mutation knobs** for stress-testing (R3+R4),
    # not as structural classification axes. Including them here would split
    # otherwise-identical scenario structures across (clear|morning) /
    # (clear|night) / (rain|*) cells purely on the mutation dimension, inflating
    # the singleton rate at the cluster-signature level. Excluded for symmetry
    # with how downstream uses environment.
    return (
        r.get("topology", "?"),
        _bin_lane(r.get("lanes")),
        sut.get("maneuver", "?"),
        _bin_npc_count(len(npcs)),
        npc_sig,
        coll_pair,
    )


def _hamming(a: tuple, b: tuple) -> float:
    # npc_sig (index 4) is a tuple → compare as set Jaccard distance for shape robustness
    diffs = 0.0
    n = len(a)
    for i in range(n):
        if i == 4:
            sa, sb = set(a[i]), set(b[i])
            if sa or sb:
                inter = len(sa & sb)
                union = len(sa | sb)
                diffs += 0.0 if union == 0 else (1.0 - inter / union)
        else:
            diffs += 0 if a[i] == b[i] else 1
    return diffs / n

def _silhouette(D: np.ndarray, labels: np.ndarray) -> float:
    n = len(labels)
    if n < 2:
        return -1.0
    uniq = np.unique(labels)
    if len(uniq) < 2 or len(uniq) >= n:
        return -1.0
    scores = []
    for i in range(n):
        own = [j for j in range(n) if labels[j] == labels[i] and j != i]
        if not own:
            scores.append(0.0)
            continue
        a = float(np.mean([D[i, j] for j in own]))
        b = float("inf")
        for c in uniq:
            if c == labels[i]:
                continue
            others = [j for j in range(n) if labels[j] == c]
            if not others:
                continue
            mean_d = float(np.mean([D[i, j] for j in others]))
            b = min(b, mean_d)
        s = 0.0 if max(a, b) == 0 else (b - a) / max(a, b)
        scores.append(s)
    return float(np.mean(scores))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--eval-set", type=Path, default=ROOT / "paper/eval_set.json")
    ap.add_argument("--road-dir", type=Path, default=ROOT / "outputs/road_seed")
    ap.add_argument("--scene-dir", type=Path, default=ROOT / "outputs/scene_seed")
    ap.add_argument("--signature-dir", type=Path, default=None,
                    help="if set, read sig_v1 signatures from here (4 scene axes) instead of full scene_seed; "
                         "topology/lanes still come from --road-dir. Use this once "
                         "outputs/signature/ has been backfilled by tools/batch_infer_signatures.py.")
    ap.add_argument("--out-medoids", type=Path, default=ROOT / "paper/eval_medoids.json")
    ap.add_argument("--out-report", type=Path, default=ROOT / "paper/eval_cluster_report.md")
    ap.add_argument("--k-min", type=int, default=2)
    ap.add_argument("--k-max", type=int, default=20,
                    help="upper bound on cluster count; silhouette will pick the best k in [k_min, min(k_max, n-1)]")
    args = ap.parse_args()

    es = json.loads(args.eval_set.read_text())
    use_sig = args.signature_dir is not None
    rows: list[dict[str, Any]] = []
    skipped: list[tuple[str, str]] = []
    for c in es["cases"]:
        cid = c["case_id"]
        rs_path = args.road_dir / f"{cid}.json"
        if not rs_path.is_file():
            skipped.append((cid, "no road_seed"))
            continue
        road = json.loads(rs_path.read_text())
        if road.get("status") != "supported":
            skipped.append((cid, f"road status={road.get('status')}"))
            continue
        if use_sig:
            sig_path = args.signature_dir / f"{cid}.json"
            if not sig_path.is_file():
                skipped.append((cid, "no signature (not inferred yet)"))
                continue
            sig_doc = json.loads(sig_path.read_text())
            if sig_doc.get("status") != "supported":
                skipped.append((cid, f"signature status={sig_doc.get('status')}"))
                continue
            rows.append({"case_id": cid, "road": road, "scene": sig_doc,
                         "feat": _feature_vector_from_signature(road, sig_doc)})
        else:
            sc_path = args.scene_dir / f"{cid}.json"
            if not sc_path.is_file():
                skipped.append((cid, "no scene_seed (not inferred yet)"))
                continue
            scene = json.loads(sc_path.read_text())
            if scene.get("status") != "supported":
                skipped.append((cid, f"scene status={scene.get('status')}"))
                continue
            rows.append({"case_id": cid, "road": road, "scene": scene,
                         "feat": _feature_vector(road, scene)})

    n = len(rows)
    print(f"eval_set total={len(es['cases'])}  clusterable={n}  skipped={len(skipped)}")
    if n < args.k_min:
        print(f"FATAL: only {n} clusterable cases — need >= {args.k_min}")
        return 2

    # Distance matrix
    D = np.zeros((n, n))
    for i in range(n):
        for j in range(i + 1, n):
            D[i, j] = D[j, i] = _hamming(rows[i]["feat"], rows[j]["feat"])

    # Sweep k, pick silhouette argmax
    Z = linkage(squareform(D, checks=False), method="average")
    best_k, best_s, best_labels = None, -2.0, None
    k_upper = min(args.k_max, n - 1)
    for k in range(args.k_min, k_upper + 1):
        labels = fcluster(Z, t=k, criterion="maxclust")
        s = _silhouette(D, labels)
        print(f"  k={k:2d}  silhouette={s:+.4f}")
        if s > best_s:
            best_k, best_s, best_labels = k, s, labels
    assert best_labels is not None

    # Pick medoids
    clusters: dict[int, list[int]] = defaultdict(list)
    for idx, lab in enumerate(best_labels):
        clusters[int(lab)].append(idx)
    medoid_records = []
    for lab, members in sorted(clusters.items()):
        if len(members) == 1:
            med = members[0]
        else:
            best_sum = float("inf")
            med = members[0]
            for cand in sorted(members, key=lambda i: rows[i]["case_id"]):
                s = sum(D[cand, m] for m in members if m != cand)
                if s < best_sum:
                    best_sum, med = s, cand
        member_cids = sorted(rows[m]["case_id"] for m in members)
        medoid_records.append({
            "cluster_id": lab,
            "size": len(members),
            "medoid": rows[med]["case_id"],
            "members": member_cids,
            "feature_signature": {
                "topology": rows[med]["feat"][0],
                "lanes": rows[med]["feat"][1],
                "sut_maneuver": rows[med]["feat"][2],
                "npc_count_bin": rows[med]["feat"][3],
                "npc_sig": list(rows[med]["feat"][4]),
                "collision_pair": rows[med]["feat"][5],
            },
        })

    args.out_medoids.parent.mkdir(parents=True, exist_ok=True)
    args.out_medoids.write_text(json.dumps({
        "version": "eval_medoids_v1",
        "source_eval_set": str(args.eval_set.relative_to(ROOT)) if args.eval_set.is_relative_to(ROOT) else str(args.eval_set),
        "clusterable_n": n,
        "skipped": [{"case_id": c, "reason": r} for c, r in skipped],
        "best_k": best_k,
        "silhouette": best_s,
        "medoids": medoid_records,
    }, indent=2, ensure_ascii=False))

    # Audit report
    lines = [
        f"# Eval-set cluster report",
        "",
        f"- source eval set: `{args.eval_set.name}` (n={len(es['cases'])})",
        f"- clusterable (supported + both seeds present): **{n}**",
        f"- skipped: {len(skipped)}",
        f"- best k by silhouette sweep [{args.k_min},{k_upper}]: **k={best_k}**, silhouette={best_s:.4f}",
        "",
        "## Skipped cases",
        ""]
    for cid, reason in skipped:
        lines.append(f"- `{cid}` — {reason}")
    lines.extend(["", "## Clusters (each row = one cluster, medoid bolded)", ""])
    for r in medoid_records:
        lines.append(f"### Cluster {r['cluster_id']} (size {r['size']}) — medoid: **`{r['medoid']}`**")
        lines.append("")
        lines.append(f"Feature signature of medoid: `{json.dumps(r['feature_signature'], ensure_ascii=False)}`")
        lines.append("")
        lines.append("Members:")
        for cid in r["members"]:
            tag = " ◀ medoid" if cid == r["medoid"] else ""
            lines.append(f"- `{cid}`{tag}")
        lines.append("")
    args.out_report.parent.mkdir(parents=True, exist_ok=True)
    args.out_report.write_text("\n".join(lines))
    print(f"\nwrote {args.out_medoids} + {args.out_report}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
