#!/usr/bin/env python3
"""Unified dashboard for the road-scene pipeline.

Two halves:
- **New run**: POST a text or PDF -> background thread runs ``coordinator.dispatch_full``
  -> pipeline phases stream over SSE -> products land in ``outputs/web_runs/<name>/``.
- **Browse**: list + detail viewer for any run dir already on disk. Detail page is
  an 11-node DAG from input through L3 behavior–intent validation; the replay node
  embeds the mp4 + SVG top-down view that the previous viewer pioneered.

    uv run python tools/web_app.py [--out-root outputs/web_runs] [--port 5000]
"""

from __future__ import annotations

import argparse
import json
import queue
import sys
import threading
import time
import uuid
from pathlib import Path
from typing import Any

from flask import (
    Flask, Response, abort, jsonify, render_template_string,
    request, send_file, stream_with_context, url_for,
)

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tools.replay_geom import parse_xodr_lines  # noqa: E402

DEFAULT_OUT_ROOT = REPO_ROOT / "outputs" / "web_runs"
DEFAULT_ENV = REPO_ROOT / ".env.local"

app = Flask(__name__)
CONFIG: dict[str, Any] = {
    "out_root": DEFAULT_OUT_ROOT,
    "model": "deepseek/deepseek-v4-pro",
    "base_url": "https://openrouter.ai/api/v1",
    "api_key_env": "OPENROUTER_API_KEY",
    "vlm_model": "xiaomi/mimo-v2.5",
}

# In-process job registry. Key is run name. Value: queue.Queue for SSE events,
# Thread object, snapshot of latest phase events for late subscribers.
JOBS: dict[str, dict[str, Any]] = {}
JOBS_LOCK = threading.Lock()

ALLOWED_UPLOAD_SUFFIXES = {".pdf", ".docx", ".pptx", ".xlsx", ".html", ".htm", ".csv", ".txt", ".md"}


# ---- helpers ----------------------------------------------------------------


def _safe_name(stem: str) -> str:
    cleaned = "".join(c if c.isalnum() or c in "-_" else "_" for c in stem).strip("_")
    return cleaned or "run"


def _run_dir(name: str) -> Path:
    out_root: Path = CONFIG["out_root"]
    candidate = (out_root / name).resolve()
    if out_root.resolve() not in candidate.parents:
        abort(404)
    if not candidate.is_dir():
        abort(404)
    return candidate


def _load_json(path: Path) -> dict[str, Any] | None:
    if not path.is_file():
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return None


def _iter_jsonl(path: Path):
    if not path.is_file():
        return
    with path.open(encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                yield json.loads(line)
            except json.JSONDecodeError:
                continue


def _scan_runs() -> list[dict[str, Any]]:
    out_root: Path = CONFIG["out_root"]
    if not out_root.is_dir():
        return []
    rows: list[dict[str, Any]] = []
    for entry in out_root.iterdir():
        if not entry.is_dir():
            continue
        summary = _load_json(entry / "summary.json") or {}
        result = _load_json(entry / "result.json") or {}
        outcome = result.get("scene_outcome") or _load_json(entry / "behavior_check.json") or {}
        readiness = result.get("demo_readiness") or {}
        has_video = (entry / "carla_rgb.mp4").is_file()
        carla_status = result.get("carla_status") or ("ok" if summary and has_video else None)
        rows.append({
            "name": entry.name,
            "mtime": entry.stat().st_mtime,
            "duration_seconds": summary.get("duration_seconds"),
            "collision_count": summary.get("collision_count"),
            "lane_invasion_count": summary.get("lane_invasion_count"),
            "termination_reason": summary.get("termination_reason"),
            "has_video": has_video,
            "has_map": (entry / "map.xodr").is_file(),
            "qa_verdict": (result.get("qa") or {}).get("final_verdict"),
            "carla_status": carla_status,
            "pcla_agent": result.get("pcla_agent"),
            "outcome_verdict": outcome.get("verdict"),
            "outcome_aligned": outcome.get("aligned"),
            "demo_ready": readiness.get("passed"),
            "is_pipeline_run": bool(result),  # vs CLI-only carla_remote run
        })
    rows.sort(key=lambda r: r["mtime"], reverse=True)
    return rows


def _frames_payload(run_dir: Path) -> dict[str, Any]:
    frames: list[dict[str, float]] = []
    t0: float | None = None
    for row in _iter_jsonl(run_dir / "frame_states.jsonl"):
        sim_t = float(row.get("simulation_time", 0.0))
        if t0 is None:
            t0 = sim_t
        frames.append({
            "t": round(sim_t - t0, 4),
            "x": float(row.get("x", 0.0)),
            "y": float(row.get("y", 0.0)),
            "yaw": float(row.get("yaw", 0.0)),
            "speed": float(row.get("speed", 0.0)),
        })
    return {"t0_sim": t0 or 0.0, "frames": frames}


def _events_payload(run_dir: Path, t0_sim: float) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    for row in _iter_jsonl(run_dir / "events.jsonl"):
        sim_t = float(row.get("simulation_time", 0.0))
        out.append({
            "t": round(sim_t - t0_sim, 4),
            "type": row.get("event_type", "?"),
            "frame": row.get("frame"),
            "payload": row.get("payload", {}),
        })
    return out


def _roads_payload(run_dir: Path) -> list[dict[str, Any]]:
    xodr_candidates = [run_dir / "map.xodr"]
    # Also try the coordinator-named XODR: <name>.xodr.
    xodr_candidates.extend(run_dir.glob("*.xodr"))
    seen = set()
    for cand in xodr_candidates:
        if cand in seen or not cand.exists():
            continue
        seen.add(cand)
        roads = parse_xodr_lines(cand)
        if roads:
            return [{"road_id": r["road_id"], "points": [[float(x), float(y)] for x, y in r["points"]]}
                    for r in roads]
    return []


# ---- pipeline job orchestration --------------------------------------------


def _start_job(name: str, source: str | Path, *,
               carla_enabled: bool, qa_max_retries: int,
               pcla_agent: str) -> None:
    """Launch dispatch_full in a daemon thread, wiring progress into a queue."""
    from tools.coordinator import dispatch_full, load_env
    load_env(DEFAULT_ENV)

    q: queue.Queue[dict[str, Any]] = queue.Queue()
    history: list[dict[str, Any]] = []

    def progress(event: str, payload: dict[str, Any]):
        msg = {"event": event, "payload": payload, "t": time.time()}
        history.append(msg)
        q.put(msg)

    def runner():
        try:
            dispatch_full(
                source, name, CONFIG["out_root"],
                model=CONFIG["model"], base_url=CONFIG["base_url"],
                api_key_env=CONFIG["api_key_env"], vlm_model=CONFIG["vlm_model"],
                qa_max_retries=qa_max_retries,
                carla_enabled=carla_enabled,
                pcla_agent=pcla_agent,
                progress=progress,
            )
        except Exception as exc:  # noqa: BLE001
            progress("fatal", {"error": f"{type(exc).__name__}: {exc}"})

    th = threading.Thread(target=runner, name=f"pipeline-{name}", daemon=True)
    with JOBS_LOCK:
        JOBS[name] = {"queue": q, "thread": th, "history": history, "done": False}
    th.start()


# ---- routes -----------------------------------------------------------------


@app.get("/")
def index():
    runs = _scan_runs()
    accepted_demo = next((run for run in runs if run.get("demo_ready") is True), None)
    return render_template_string(INDEX_HTML, runs=runs, accepted_demo=accepted_demo)


@app.get("/new")
def new_form():
    demo_path = REPO_ROOT / "inputs" / "demo_case165.txt"
    demo_text = demo_path.read_text(encoding="utf-8") if demo_path.is_file() else ""
    return render_template_string(NEW_HTML, demo_text=demo_text)


@app.post("/new")
def new_submit():
    upload = request.files.get("file")
    text = (request.form.get("text") or "").strip()
    explicit_name = (request.form.get("name") or "").strip()
    carla_enabled = request.form.get("carla_enabled", "") in ("on", "1", "true", "yes")
    qa_max_retries = int(request.form.get("qa_max_retries", "1"))
    pcla_agent = request.form.get("pcla_agent", "if_if")
    if pcla_agent not in {"if_if", "tfv6_regnet"}:
        return jsonify(ok=False, error=f"unsupported ADS agent: {pcla_agent}"), 400

    if upload and upload.filename:
        suffix = Path(upload.filename).suffix.lower()
        if suffix not in ALLOWED_UPLOAD_SUFFIXES:
            return jsonify(ok=False, error=f"unsupported file type: {suffix}"), 400
        name = _safe_name(explicit_name or Path(upload.filename).stem)
        run_dir = CONFIG["out_root"] / name
        run_dir.mkdir(parents=True, exist_ok=True)
        source_path = run_dir / f"source{suffix}"
        upload.save(str(source_path))
        source: str | Path = source_path
    elif text:
        name = _safe_name(explicit_name or f"text_{uuid.uuid4().hex[:6]}")
        # Spill the raw paragraph to a .txt file rather than passing the string
        # directly: extract_source.extract_any rejects strings containing '/'
        # or known suffixes, which trips up most accident narratives.
        run_dir = CONFIG["out_root"] / name
        run_dir.mkdir(parents=True, exist_ok=True)
        source_path = run_dir / "source.txt"
        source_path.write_text(text, encoding="utf-8")
        source = source_path
    else:
        return jsonify(ok=False, error="provide either a file or text"), 400

    with JOBS_LOCK:
        if name in JOBS and not JOBS[name].get("done"):
            return jsonify(ok=False, error=f"job '{name}' is already running"), 409

    _start_job(name, source, carla_enabled=carla_enabled,
               qa_max_retries=qa_max_retries, pcla_agent=pcla_agent)
    return jsonify(ok=True, name=name, detail_url=url_for("detail", name=name, live=1))


@app.get("/pipeline/<name>/stream")
def stream(name: str):
    with JOBS_LOCK:
        job = JOBS.get(name)
    if job is None:
        abort(404)

    @stream_with_context
    def gen():
        # Replay history so a late subscriber sees what already happened.
        for msg in list(job["history"]):
            yield _format_sse(msg)
        q: queue.Queue = job["queue"]
        while True:
            try:
                msg = q.get(timeout=20)
            except queue.Empty:
                yield ": keep-alive\n\n"
                if job.get("done"):
                    break
                continue
            yield _format_sse(msg)
            if msg["event"] in {"done", "fatal"}:
                job["done"] = True
                break

    return Response(gen(), mimetype="text/event-stream")


def _format_sse(msg: dict[str, Any]) -> str:
    data = json.dumps(msg.get("payload", {}), ensure_ascii=False)
    return f"event: {msg['event']}\ndata: {data}\n\n"


@app.get("/run/<name>")
def detail(name: str):
    run_dir = _run_dir(name)
    result = _load_json(run_dir / "result.json") or {}
    summary = _load_json(run_dir / "summary.json") or {}
    frames_blob = _frames_payload(run_dir)
    events = _events_payload(run_dir, frames_blob["t0_sim"])
    roads = _roads_payload(run_dir)

    live = request.args.get("live") == "1"
    job_active = False
    with JOBS_LOCK:
        if name in JOBS and not JOBS[name].get("done"):
            job_active = True

    return render_template_string(
        DETAIL_HTML,
        name=name,
        summary=summary,
        result=result,
        has_video=(run_dir / "carla_rgb.mp4").is_file(),
        has_map=bool(roads),
        run_data_json=json.dumps({
            "frames": frames_blob["frames"],
            "events": events,
            "roads": roads,
        }, ensure_ascii=False),
        result_json=json.dumps(result, ensure_ascii=False),
        live=live,
        job_active=job_active,
    )


@app.get("/run/<name>/video")
def run_video(name: str):
    mp4 = _run_dir(name) / "carla_rgb.mp4"
    if not mp4.is_file():
        abort(404)
    return send_file(str(mp4), mimetype="video/mp4", conditional=True)


@app.get("/run/<name>/frames.json")
def run_frames(name: str):
    return jsonify(_frames_payload(_run_dir(name)))


@app.get("/run/<name>/events.json")
def run_events(name: str):
    run_dir = _run_dir(name)
    return jsonify(_events_payload(run_dir, _frames_payload(run_dir)["t0_sim"]))


@app.get("/run/<name>/summary.json")
def run_summary(name: str):
    return jsonify(_load_json(_run_dir(name) / "summary.json") or {})


@app.get("/run/<name>/result.json")
def run_result(name: str):
    return jsonify(_load_json(_run_dir(name) / "result.json") or {})


@app.get("/run/<name>/roads.json")
def run_roads(name: str):
    return jsonify(_roads_payload(_run_dir(name)))


@app.get("/run/<name>/xodr-preview")
def run_xodr_preview(name: str):
    """Serve the road-only HTML preview generated by build_road_seed_opendrive."""
    run_dir = _run_dir(name)
    candidates = sorted(run_dir.glob("*.html"))
    # Filter out anything that obviously isn't the road preview (defensive).
    html = next((c for c in candidates if c.stem == name), candidates[0] if candidates else None)
    if not html:
        abort(404)
    return send_file(str(html))


@app.get("/run/<name>/qa-artifact/<int:round_idx>/<kind>")
def qa_artifact(name: str, round_idx: int, kind: str):
    run_dir = _run_dir(name)
    filename = {
        "composite": f"qa_composite_r{round_idx}.png",
        "map":       f"qa_map_r{round_idx}.png",
        "scene":     f"qa_scene_r{round_idx}.png",
    }.get(kind)
    if not filename:
        abort(400)
    path = run_dir / "qa" / f"r{round_idx}" / filename
    if not path.exists():
        abort(404)
    return send_file(str(path), mimetype="image/png")


@app.get("/run/<name>/modeling/<path:filename>")
def modeling_artifact(name: str, filename: str):
    """Serve modeling/ artifacts (puml / mmd / json) for inline rendering."""
    if "/" in filename or filename.startswith(".."):
        abort(400)
    path = _run_dir(name) / "modeling" / filename
    if not path.is_file():
        abort(404)
    if filename.endswith(".json"):
        mime = "application/json"
    elif filename.endswith(".mmd") or filename.endswith(".puml"):
        mime = "text/plain"
    else:
        mime = "application/octet-stream"
    return Response(path.read_text(encoding="utf-8"), mimetype=f"{mime}; charset=utf-8")


# ---- templates --------------------------------------------------------------

_STYLE_BASE = """
  :root {
    --bd:#1f2940; --bd-strong:#2a3756;
    --bg:#0a0e1a; --bg-elev:#0e1421;
    --card:#131826; --card-hi:#1a2138;
    --ink:#e6e9f0; --sub:#8b95ab;
    --accent:#00d4aa; --accent-soft:rgba(0,212,170,0.13); --accent-glow:rgba(0,212,170,0.40);
    --ok:#00d4aa; --ok-soft:rgba(0,212,170,0.13);
    --warn:#f59e0b; --warn-soft:rgba(245,158,11,0.14);
    --bad:#ef4444; --bad-soft:rgba(239,68,68,0.14);
    --idle:#4a5577; --idle-soft:rgba(74,85,119,0.18);
  }
  * { box-sizing:border-box; }
  body { margin:0; font-family:-apple-system,BlinkMacSystemFont,"SF Pro Text","Segoe UI","PingFang SC",sans-serif; color:var(--ink); background:var(--bg); -webkit-font-smoothing:antialiased; }
  header { padding:14px 26px; background:linear-gradient(180deg,#0c1220 0%,#0a0e1a 100%); border-bottom:1px solid var(--bd); color:#fff; display:flex; align-items:baseline; gap:16px; }
  header h1 { margin:0; font-size:16px; font-weight:600; letter-spacing:-0.01em; }
  header a { color:var(--sub); font-size:12.5px; text-decoration:none; }
  header a:hover { color:var(--accent); }
  main { max-width:1380px; margin:0 auto; padding:22px 26px 40px; }
  .pill { display:inline-block; padding:2px 9px; border-radius:999px; font-size:11.5px; font-weight:600; background:var(--idle-soft); color:var(--idle); }
  .pill.ok { background:var(--ok-soft); color:var(--ok); }
  .pill.warn { background:var(--warn-soft); color:var(--warn); }
  .pill.bad { background:var(--bad-soft); color:var(--bad); }
"""

INDEX_HTML = """<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8" /><title>Crash2OpenX · Dashboard</title>
<style>__STYLE__
  .topbar { display:flex; justify-content:space-between; align-items:center; margin-bottom:14px; }
  .new-btn { background:var(--accent); color:#fff; padding:9px 18px; border-radius:8px; text-decoration:none; font-weight:600; font-size:13.5px; }
  .new-btn:hover { background:#3f5de0; }
  .demo-banner { display:flex; align-items:center; justify-content:space-between; gap:18px; margin:0 0 16px; padding:16px 18px; border:1px solid #b8ead8; border-radius:12px; background:linear-gradient(135deg,#f0fcf7,#f7fbff); box-shadow:0 1px 3px rgba(20,30,60,.04); }
  .demo-banner h2 { margin:0 0 5px; font-size:15px; }
  .demo-banner p { margin:0; color:var(--sub); font-size:12.5px; }
  .demo-open { flex:none; color:#fff; background:#11865f; padding:9px 15px; border-radius:8px; text-decoration:none; font-size:13px; font-weight:650; }
  .demo-open:hover { background:#0d704f; }
  table { width:100%; border-collapse:separate; border-spacing:0; background:#fff; border:1px solid var(--bd); border-radius:12px; overflow:hidden; box-shadow:0 1px 3px rgba(20,30,60,.04); }
  th, td { padding:11px 14px; text-align:left; font-size:13.5px; }
  th { background:#f8fafc; color:var(--sub); font-weight:600; text-transform:uppercase; letter-spacing:.5px; font-size:11.5px; border-bottom:1px solid var(--bd); }
  tr + tr td { border-top:1px solid #eef1f6; }
  td a { color:var(--accent); text-decoration:none; font-weight:600; }
  td a:hover { text-decoration:underline; }
  .empty { padding:48px; text-align:center; color:var(--sub); }
</style></head>
<body>
<header><h1>Crash2OpenX · Dashboard</h1><span style="color:#9aa4b5;font-size:12.5px">NL → seed → XODR → QA → XOSC → CARLA → L3 intent → replay</span></header>
<main>
  {% if accepted_demo %}
  <div class="demo-banner">
    <div>
      <h2><span class="pill ok">PASS</span> Accepted 5-minute demo · {{ accepted_demo.name }}</h2>
      <p>{{ accepted_demo.pcla_agent or '—' }} · {{ accepted_demo.collision_count or 0 }} collision · L3 {{ accepted_demo.outcome_verdict or '—' }} · {{ '%.1f s' % accepted_demo.duration_seconds if accepted_demo.duration_seconds is not none else '—' }}</p>
    </div>
    <a class="demo-open" href="/run/{{ accepted_demo.name }}">打开验收演示 →</a>
  </div>
  {% endif %}
  <div class="topbar">
    <div style="color:var(--sub);font-size:13px">outputs/web_runs/ · 共 {{ runs|length }} 个 run</div>
    <a class="new-btn" href="/new">+ 新建 run</a>
  </div>
  {% if runs %}
  <table>
    <thead><tr><th>Run</th><th>Demo gate</th><th>Pipeline</th><th>QA</th><th>CARLA</th><th>ADS</th><th>L3 intent</th><th>Duration</th><th>Collisions</th><th>Lane invs</th><th>Term</th><th>Video</th></tr></thead>
    <tbody>
    {% for r in runs %}
      <tr>
        <td><a href="/run/{{ r.name }}">{{ r.name }}</a></td>
        <td>
          {% if r.demo_ready is sameas true %}<span class="pill ok">PASS</span>
          {% elif r.demo_ready is sameas false %}<span class="pill bad">FAIL</span>
          {% else %}<span class="pill">—</span>{% endif %}
        </td>
        <td>{% if r.is_pipeline_run %}<span class="pill ok">✓</span>{% else %}<span class="pill">carla only</span>{% endif %}</td>
        <td>
          {% if r.qa_verdict == 'pass' %}<span class="pill ok">pass</span>
          {% elif r.qa_verdict in ('fail', 'error') %}<span class="pill bad">{{ r.qa_verdict }}</span>
          {% elif r.qa_verdict == 'skipped' %}<span class="pill warn">skipped</span>
          {% else %}<span class="pill">—</span>{% endif %}
        </td>
        <td>
          {% if r.carla_status == 'ok' %}<span class="pill ok">ok</span>
          {% elif r.carla_status and r.carla_status.startswith('skipped') %}<span class="pill warn">{{ r.carla_status[8:] }}</span>
          {% elif r.carla_status and r.carla_status.startswith('error') %}<span class="pill bad">error</span>
          {% else %}<span class="pill">—</span>{% endif %}
        </td>
        <td><span class="pill">{{ r.pcla_agent or '—' }}</span></td>
        <td>
          {% if r.outcome_aligned is sameas true %}<span class="pill ok">{{ r.outcome_verdict or 'aligned' }}</span>
          {% elif r.outcome_aligned is sameas false %}<span class="pill bad">{{ r.outcome_verdict or 'drifted' }}</span>
          {% elif r.outcome_verdict %}<span class="pill warn">{{ r.outcome_verdict }}</span>
          {% else %}<span class="pill">—</span>{% endif %}
        </td>
        <td>{{ '%.1f s' % r.duration_seconds if r.duration_seconds is not none else '—' }}</td>
        <td>{% set c = r.collision_count %}<span class="pill {{ 'ok' if r.outcome_aligned is sameas true else ('bad' if c and c > 0 else ('warn' if c == 0 and r.outcome_aligned is sameas false else '')) }}">{{ c if c is not none else '—' }}</span></td>
        <td>{% set l = r.lane_invasion_count %}<span class="pill {{ 'warn' if l and l > 0 else ('ok' if l == 0 else '') }}">{{ l if l is not none else '—' }}</span></td>
        <td>{{ r.termination_reason or '—' }}</td>
        <td><span class="pill {{ 'ok' if r.has_video else '' }}">{{ '✓' if r.has_video else '—' }}</span></td>
      </tr>
    {% endfor %}
    </tbody>
  </table>
  {% else %}
  <div class="empty">还没有 run。点右上「+ 新建 run」开始。</div>
  {% endif %}
</main>
</body></html>
""".replace("__STYLE__", _STYLE_BASE)


NEW_HTML = """<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8" /><title>新建 run · Crash2OpenX</title>
<style>__STYLE__
  .card { background:var(--card); border:1px solid var(--bd); border-radius:12px; padding:22px; box-shadow:0 1px 3px rgba(20,30,60,.04); }
  h2 { margin:0 0 16px; font-size:15.5px; }
  .tabs { display:flex; gap:8px; margin-bottom:10px; }
  .tab { padding:7px 14px; border:1px solid var(--bd); border-radius:999px; cursor:pointer; background:#fbfcfe; font-size:13px; }
  .tab.active { border-color:var(--accent); color:var(--accent); background:var(--accent-soft); font-weight:600; }
  .preset { margin-left:auto; border:1px solid var(--accent); color:var(--accent); background:var(--accent-soft); border-radius:999px; padding:7px 13px; cursor:pointer; font-size:12px; font-weight:600; }
  textarea { width:100%; min-height:160px; padding:12px; border:1px solid var(--bd); border-radius:10px; font-size:13px; font-family:inherit; resize:vertical; }
  textarea:focus, input:focus, select:focus { outline:none; border-color:var(--accent); box-shadow:0 0 0 3px var(--accent-soft); }
  input[type=text], input[type=number], select { width:100%; padding:9px 11px; border:1px solid var(--bd); border-radius:10px; font-size:13px; background:var(--card); color:var(--ink); }
  .file-drop { border:1.5px dashed var(--bd); border-radius:10px; padding:32px 14px; text-align:center; color:var(--sub); font-size:13px; cursor:pointer; }
  .row { display:grid; grid-template-columns:1fr 220px; gap:18px; margin-top:12px; }
  label.sm { font-size:12px; color:var(--sub); display:block; margin-bottom:4px; }
  .opts { display:flex; flex-direction:column; gap:10px; }
  .checkbox { display:flex; align-items:center; gap:8px; font-size:13px; }
  button.go { width:100%; padding:11px 16px; border:0; border-radius:10px; background:var(--accent); color:#fff; font-size:14px; font-weight:600; cursor:pointer; }
  button.go:disabled { opacity:.55; }
  #status { font-size:12px; color:var(--sub); text-align:center; min-height:16px; margin-top:8px; }
  .hidden { display:none; }
  .err { color:var(--bad); }
</style></head>
<body>
<header><h1>新建 run</h1><a href="/">← 返回列表</a></header>
<main>
  <div class="card">
    <h2>📄 输入</h2>
    <div class="tabs">
      <div class="tab active" data-tab="text">文字</div>
      <div class="tab" data-tab="file">上传 PDF / 文档</div>
      <button class="preset" type="button" id="load-demo">载入 5 分钟 demo case</button>
    </div>
    <div id="pane-text">
      <textarea id="text" placeholder="贴一段交通事故描述（中/英文均可）…"></textarea>
    </div>
    <div id="pane-file" class="hidden">
      <label class="file-drop" for="file">
        <input type="file" id="file" accept=".pdf,.docx,.pptx,.xlsx,.html,.htm,.csv,.txt,.md" style="display:none" />
        <span id="file-label">点击选择 · PDF / Office / txt / md</span>
      </label>
    </div>
    <div class="row">
      <div class="opts">
        <div>
          <label class="sm">run 名（可选）</label>
          <input type="text" id="name" placeholder="留空自动生成" />
        </div>
        <div class="checkbox">
          <input type="checkbox" id="carla_enabled" checked />
          <label for="carla_enabled">QA 通过后跑 CARLA（≈ 1–2 分钟）</label>
        </div>
        <div>
          <label class="sm">QA 最大重试轮数</label>
          <input type="number" id="qa_max_retries" value="1" min="0" max="3" />
        </div>
        <div>
          <label class="sm">被测 ADS agent</label>
          <select id="pcla_agent">
            <option value="if_if" selected>InterFuser (if_if) · demo 推荐</option>
            <option value="tfv6_regnet">TransFuser++ (tfv6_regnet) · 对照</option>
          </select>
        </div>
      </div>
      <div style="display:flex;flex-direction:column;justify-content:flex-end">
        <button class="go" id="go">运行流水线</button>
        <div id="status"></div>
      </div>
    </div>
  </div>
</main>
<script>
const DEMO_CASE_TEXT = {{ demo_text|tojson }};
let currentTab = "text";
document.querySelectorAll(".tab").forEach(t => {
  t.onclick = () => {
    currentTab = t.dataset.tab;
    document.querySelectorAll(".tab").forEach(x => x.classList.toggle("active", x === t));
    document.getElementById("pane-text").classList.toggle("hidden", currentTab !== "text");
    document.getElementById("pane-file").classList.toggle("hidden", currentTab !== "file");
  };
});
document.getElementById("file").addEventListener("change", e => {
  const f = e.target.files[0];
  document.getElementById("file-label").textContent = f ? f.name : "点击选择 · PDF / Office / txt / md";
});
document.getElementById("load-demo").onclick = () => {
  currentTab = "text";
  document.querySelectorAll(".tab").forEach(x => x.classList.toggle("active", x.dataset.tab === "text"));
  document.getElementById("pane-text").classList.remove("hidden");
  document.getElementById("pane-file").classList.add("hidden");
  document.getElementById("text").value = DEMO_CASE_TEXT;
  document.getElementById("name").value = `demo_case165_${new Date().toISOString().slice(0,19).replace(/[-:T]/g,'')}`;
  document.getElementById("pcla_agent").value = "if_if";
};

document.getElementById("go").onclick = async () => {
  const btn = document.getElementById("go");
  const status = document.getElementById("status");
  const fd = new FormData();
  const name = document.getElementById("name").value.trim();
  if (name) fd.append("name", name);
  fd.append("carla_enabled", document.getElementById("carla_enabled").checked ? "on" : "");
  fd.append("qa_max_retries", document.getElementById("qa_max_retries").value);
  fd.append("pcla_agent", document.getElementById("pcla_agent").value);

  if (currentTab === "text") {
    const text = document.getElementById("text").value.trim();
    if (!text) { alert("请输入文字"); return; }
    fd.append("text", text);
  } else {
    const f = document.getElementById("file").files[0];
    if (!f) { alert("请选择文件"); return; }
    fd.append("file", f);
  }
  btn.disabled = true; status.textContent = "启动中…";
  try {
    const r = await fetch("/new", { method:"POST", body: fd });
    const d = await r.json();
    if (!d.ok) { status.innerHTML = `<span class="err">${d.error}</span>`; btn.disabled = false; return; }
    window.location = d.detail_url;
  } catch (e) {
    status.innerHTML = `<span class="err">请求失败: ${e}</span>`;
    btn.disabled = false;
  }
};
</script>
</body></html>
""".replace("__STYLE__", _STYLE_BASE)


DETAIL_HTML = """<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8" /><title>{{ name }} · Detail</title>
<script src="https://cdn.jsdelivr.net/npm/mermaid@10/dist/mermaid.min.js"></script>
<style>__STYLE__
  /* === Detail page · dark theme with SVG pipeline === */
  .live-banner { background:var(--accent-soft); color:var(--accent); padding:8px 14px; border-radius:8px; font-size:12.5px; margin-bottom:14px; display:flex; align-items:center; gap:10px; border:1px solid rgba(0,212,170,0.25); }
  .live-dot { width:8px; height:8px; border-radius:50%; background:var(--accent); animation:pulse 1.4s ease-in-out infinite; box-shadow:0 0 0 4px var(--accent-soft); }
  @keyframes pulse { 0%,100%{opacity:1;transform:scale(1);} 50%{opacity:.5;transform:scale(1.4);} }

  /* Two-column shell */
  .layout-shell { display:grid; grid-template-columns:300px minmax(0,1fr); gap:22px; align-items:flex-start; }
  .side-dag { position:sticky; top:18px; }
  .side-dag-inner { background:var(--card); border:1px solid var(--bd); border-radius:14px; padding:18px 14px 22px; box-shadow:0 10px 36px rgba(0,0,0,0.32); }
  .side-dag-head { display:flex; justify-content:space-between; align-items:baseline; padding:0 4px 14px; border-bottom:1px solid var(--bd); margin-bottom:12px; }
  .side-dag-title { font-size:10.5px; font-weight:700; letter-spacing:.8px; text-transform:uppercase; color:var(--sub); }
  .side-dag-meta { font-family:ui-monospace,Menlo,monospace; font-size:11px; color:var(--accent); }

  /* SVG pipeline (left sidebar) */
  .pipeline-svg { display:block; width:100%; height:auto; overflow:visible; }
  .svg-node { cursor:pointer; }
  .svg-node-bg { fill:var(--card-hi); stroke:var(--bd-strong); stroke-width:1; transition:.18s; }
  .svg-node:hover .svg-node-bg { fill:#1e2640; stroke:var(--accent); }
  .svg-node.active .svg-node-bg { fill:rgba(0,212,170,0.10); stroke:var(--accent); stroke-width:1.5; filter:drop-shadow(0 0 8px var(--accent-glow)); }
  .svg-node-num { font-size:9.5px; font-weight:700; fill:var(--sub); font-family:ui-monospace,Menlo,monospace; pointer-events:none; }
  .svg-node.ok .svg-node-num { fill:var(--ok); }
  .svg-node.running .svg-node-num { fill:var(--accent); }
  .svg-node.bad .svg-node-num { fill:var(--bad); }
  .svg-node.warn .svg-node-num { fill:var(--warn); }
  .svg-node-title { font-size:11.5px; font-weight:600; fill:var(--ink); pointer-events:none; }
  .svg-node-status { font-size:8.5px; fill:var(--sub); font-weight:700; text-transform:uppercase; pointer-events:none; letter-spacing:.5px; }
  .svg-node.ok .svg-node-status { fill:var(--ok); }
  .svg-node.bad .svg-node-status { fill:var(--bad); }
  .svg-node.warn .svg-node-status { fill:var(--warn); }
  .svg-node.running .svg-node-status { fill:var(--accent); }
  .svg-node-dur { font-size:9.5px; fill:var(--sub); font-family:ui-monospace,Menlo,monospace; pointer-events:none; }
  .svg-edge { fill:none; stroke:var(--bd-strong); stroke-width:1.5; }
  .svg-edge.flow-done { stroke:var(--accent); opacity:0.55; }
  .svg-edge-arrow { fill:var(--bd-strong); }
  .svg-edge-arrow.flow-done { fill:var(--accent); opacity:0.7; }

  /* Right panel · single exclusive card with empty state */
  .main-panel { min-width:0; }
  .active-panel { background:var(--card); border:1px solid var(--bd); border-radius:14px; padding:28px 30px; box-shadow:0 10px 36px rgba(0,0,0,0.32); position:relative; min-height:560px; transition:box-shadow .35s; }
  .active-panel.flash { box-shadow:0 0 0 1px var(--accent), 0 0 36px var(--accent-glow), 0 10px 36px rgba(0,0,0,0.32); }
  .active-panel .card-head { display:flex; align-items:center; gap:14px; padding-bottom:16px; border-bottom:1px solid var(--bd); margin-bottom:20px; }
  .active-panel .card-head .node-ico { font-size:24px; }
  .active-panel .card-head h3 { margin:0; font-size:18px; font-weight:600; flex:1; letter-spacing:-0.01em; color:var(--ink); }
  .active-panel .card-head .badge { padding:3px 11px; border-radius:999px; font-size:10.5px; font-weight:700; background:var(--idle-soft); color:var(--idle); text-transform:uppercase; letter-spacing:.5px; }
  .active-panel .card-head .badge.ok { background:var(--ok-soft); color:var(--ok); }
  .active-panel .card-head .badge.warn { background:var(--warn-soft); color:var(--warn); }
  .active-panel .card-head .badge.bad { background:var(--bad-soft); color:var(--bad); }
  .active-panel .card-head .badge.running { background:var(--accent-soft); color:var(--accent); }

  /* Empty state */
  .empty-state { padding:140px 30px; text-align:center; color:var(--sub); }
  .empty-state .empty-icon { font-size:54px; opacity:0.22; margin-bottom:20px; line-height:1; }
  .empty-state .empty-title { font-size:16px; color:var(--ink); margin-bottom:8px; font-weight:600; letter-spacing:-0.01em; }
  .empty-state .empty-hint { font-size:12.5px; line-height:1.7; }
  .empty-state .empty-hint kbd { background:var(--card-hi); border:1px solid var(--bd); padding:1px 7px; border-radius:5px; font-family:ui-monospace,Menlo,monospace; font-size:11px; color:var(--accent); }

  /* Content blocks */
  .section { margin-bottom:18px; }
  .section-label { font-size:10.5px; font-weight:700; color:var(--sub); text-transform:uppercase; letter-spacing:.7px; margin-bottom:8px; }
  .reason-box { background:var(--accent-soft); border-left:3px solid var(--accent); border-radius:8px; padding:11px 14px; font-size:13px; line-height:1.6; color:var(--ink); }
  pre.code { margin:0; padding:14px; font-size:12px; line-height:1.55; overflow:auto; max-height:440px; background:#070a14; color:#9eb3d4; border:1px solid var(--bd); border-radius:10px; font-family:ui-monospace,SFMono-Regular,Menlo,monospace; }
  pre.text { margin:0; padding:14px; font-size:12.5px; line-height:1.65; overflow:auto; max-height:440px; background:var(--bg-elev); border:1px solid var(--bd); border-radius:10px; white-space:pre-wrap; word-break:break-word; color:var(--ink); }
  iframe.preview { width:100%; height:540px; border:1px solid var(--bd); border-radius:10px; background:var(--bg-elev); filter:invert(0.92) hue-rotate(180deg); }
  .metrics { display:grid; grid-template-columns:repeat(auto-fit,minmax(150px,1fr)); gap:10px; margin-bottom:14px; }
  .metric { background:var(--bg-elev); border:1px solid var(--bd); border-radius:10px; padding:11px 13px; }
  .metric .lbl { font-size:10px; color:var(--sub); text-transform:uppercase; letter-spacing:.6px; }
  .metric .val { font-size:19px; font-weight:700; margin-top:4px; color:var(--ink); font-feature-settings:"tnum" 1; }
  .metric .val.bad { color:var(--bad); } .metric .val.warn { color:var(--warn); } .metric .val.ok { color:var(--ok); }
  .replay-split { display:grid; grid-template-columns:minmax(0,1.05fr) minmax(0,1fr); gap:14px; }
  video { width:100%; display:block; background:#000; border-radius:10px; }
  svg.topdown { display:block; width:100%; height:380px; background:var(--bg-elev); border:1px solid var(--bd); border-radius:10px; }
  .road-body { fill:none; stroke:#5a657a; stroke-width:6; stroke-linecap:round; }
  .traj { fill:none; stroke:#5a657a; stroke-width:1.1; }
  .start { fill:#3b82f6; } .goal { fill:var(--ok); } .live { fill:var(--bad); stroke:#fff; stroke-width:.5; }
  .ev-dot.lane_invasion { fill:var(--warn); }
  .ev-dot.collision { fill:var(--bad); }
  .ev-dot.episode_started, .ev-dot.episode_finished { fill:#3b82f6; }
  .ev-dot { stroke:var(--bg); stroke-width:.4; }
  .clock { font-family:ui-monospace,SFMono-Regular,Menlo,monospace; font-size:12px; color:var(--sub); margin-top:6px; text-align:right; }
  .qa-img { width:100%; border:1px solid var(--bd); border-radius:10px; margin:6px 0; }
  .empty { padding:32px 14px; text-align:center; color:var(--sub); font-size:13px; }
  .err { color:var(--bad); }
  .round-dot { width:7px; height:7px; border-radius:50%; background:var(--idle-soft); border:1px solid var(--idle); cursor:pointer; transition:.15s; }
  .round-dot:hover { transform:scale(1.4); }
  .round-dot.ok { background:var(--ok); border-color:var(--ok); }
  .round-dot.bad { background:var(--bad); border-color:var(--bad); }
  .round-dot.warn { background:var(--warn); border-color:var(--warn); }
  .round-dot.running { background:var(--accent); border-color:var(--accent); animation:pulse 1.2s ease-in-out infinite; }

  @media (max-width: 900px) {
    .layout-shell { grid-template-columns:1fr; }
    .side-dag { position:static; }
  }
</style></head>
<body>
<header><h1>{{ name }}</h1><a href="/">← 返回列表</a></header>
<main>
  {% if live and job_active %}
  <div class="live-banner"><div class="live-dot"></div><span>流水线运行中… SSE 实时刷新各节点状态</span></div>
  {% endif %}
  <div class="layout-shell">
    <aside class="side-dag">
      <div class="side-dag-inner">
        <div class="side-dag-head">
          <span class="side-dag-title">Pipeline</span>
          <span class="side-dag-meta" id="gantt-total"></span>
        </div>
        <svg class="pipeline-svg" id="pipeline-svg" viewBox="0 0 270 845" preserveAspectRatio="xMidYMin meet">
          <defs>
            <marker id="arr-idle" markerWidth="6" markerHeight="6" refX="5" refY="3" orient="auto">
              <path d="M0,0 L5,3 L0,6 z" class="svg-edge-arrow" />
            </marker>
            <marker id="arr-done" markerWidth="6" markerHeight="6" refX="5" refY="3" orient="auto">
              <path d="M0,0 L5,3 L0,6 z" class="svg-edge-arrow flow-done" />
            </marker>
          </defs>
          <g id="svg-edges"></g>
          <g id="svg-nodes"></g>
        </svg>
      </div>
    </aside>
    <div class="main-panel">
      <div class="active-panel" id="active-panel">
        <div class="empty-state">
          <div class="empty-icon">◌</div>
          <div class="empty-title">尚未选择阶段</div>
          <div class="empty-hint">点击左侧流程图任一节点<br/>查看该阶段产物、耗时与验证状态</div>
        </div>
      </div>
    </div>
  </div>
</main>

<script id="run-data" type="application/json">{{ run_data_json|safe }}</script>
<script id="result-data" type="application/json">{{ result_json|safe }}</script>
<script>
const RUN_NAME = {{ name|tojson }};
const RUN_DATA = JSON.parse(document.getElementById("run-data").textContent);
let RESULT = JSON.parse(document.getElementById("result-data").textContent);
const JOB_ACTIVE = {{ 'true' if job_active else 'false' }};

const STAGES = [
  { key:"input",     title:"PDF / 文本输入",        ico:"📄", x:135, y:30 },
  { key:"road",      title:"道路推理 road_seed",   ico:"🛣️", x:60,  y:115 },
  { key:"scene",     title:"场景推理 scene_seed",  ico:"🎬", x:210, y:115 },
  { key:"modeling",  title:"场景形式化 (UML)",     ico:"📐", x:210, y:200 },
  { key:"xodr",      title:"XODR 编译",             ico:"🗺️", x:60,  y:200 },
  { key:"qa",        title:"VLM QA 评审",           ico:"🔍", x:135, y:285 },
  { key:"roadgraph", title:"CARLA 取 waypoint",     ico:"🛰️", x:135, y:370 },
  { key:"xosc",      title:"XOSC 编译",             ico:"🎯", x:135, y:455 },
  { key:"carla",     title:"CARLA 跑场景",          ico:"🚗", x:135, y:540 },
  { key:"scene_outcome", title:"L3 行为—意图门",    ico:"✅", x:135, y:625 },
  { key:"replay",    title:"Replay 回放",           ico:"📺", x:135, y:710 },
];
// SVG node box dimensions
const SVG_NW = 110, SVG_NH = 56;

const NODE_STATE = {};  // key -> { status: 'idle'|'running'|'ok'|'warn'|'bad', note }

function deriveStatesFromResult() {
  if (!RESULT || Object.keys(RESULT).length === 0) {
    for (const s of STAGES) NODE_STATE[s.key] = { status: "idle", note: "" };
    return;
  }
  const r = RESULT;
  NODE_STATE.input  = { status: r.text_chars ? "ok" : "bad",   note: r.text_chars ? `${r.text_chars} 字` : "无文本" };
  NODE_STATE.road   = { status: r.road_status === "supported" ? "ok" : (r.errors && r.errors.road ? "bad" : "warn"), note: r.road_status || "—" };
  NODE_STATE.scene  = { status: r.scene_status === "supported" ? "ok" : (r.errors && r.errors.scene ? "bad" : "warn"), note: r.scene_status || "—" };
  const ms = r.modeling_status;
  let msStatus = "idle";
  if (ms === "ok") msStatus = "ok";
  else if (ms && ms.startsWith && ms.startsWith("error")) msStatus = "bad";
  else if (ms && ms.startsWith && ms.startsWith("skipped")) msStatus = "warn";
  const msSum = r.scene_model_summary;
  const msNote = ms === "ok" && msSum
      ? `${msSum.scenario_type} · A${msSum.actor_count}/R${msSum.relation_count}/P${msSum.phase_count}`
      : (ms || "—");
  NODE_STATE.modeling = { status: msStatus, note: msNote };
  NODE_STATE.xodr   = { status: r.xodr_path ? "ok" : (r.errors && r.errors.compile ? "bad" : "warn"), note: r.xodr_path ? "已生成" : (r.road_skipped || "未生成") };
  const v = r.qa && r.qa.final_verdict;
  NODE_STATE.qa     = { status: v === "pass" ? "ok" : ((v === "fail" || v === "error") ? "bad" : "warn"), note: v ? `${v} · ${r.qa.rounds_run}/${(r.qa.max_retries||0)+1} 轮` : "—" };
  const rg = r.roadgraph_status;
  NODE_STATE.roadgraph = { status: rg === "ok" ? "ok" : (rg && rg.startsWith && rg.startsWith("error") ? "bad" : (rg && rg.startsWith && rg.startsWith("skipped") ? "warn" : "idle")), note: rg || "—" };
  const xs = r.xosc_status;
  NODE_STATE.xosc   = { status: xs === "ok" ? "ok" : (xs && xs.startsWith("error") ? "bad" : "warn"), note: xs || "—" };
  const cs = r.carla_status;
  NODE_STATE.carla  = { status: cs === "ok" ? "ok" : (cs && cs.startsWith("error") ? "bad" : (cs === "pending" ? "idle" : "warn")), note: cs === "ok" && r.pcla_agent ? `${cs} · ${r.pcla_agent}` : (cs || "—") };
  const so = r.scene_outcome || {};
  NODE_STATE.scene_outcome = {
    status: so.aligned === true ? ((so.warnings||[]).length ? "warn" : "ok") : (so.aligned === false ? "bad" : (so.verdict ? "warn" : "idle")),
    note: so.verdict ? `${so.verdict}${(so.warnings||[]).length ? ' · warnings' : ''}` : "未评判",
  };
  NODE_STATE.replay = { status: cs === "ok" ? (so.aligned === false ? "warn" : "ok") : "idle", note: cs === "ok" ? "可看" : "未就绪" };
}

let activeStage = null;
let activeRound = -1;  // -1 = "show latest"

// Layout columns for the DAG grid:
// input centered (col 2), road | scene parallel (col 1 / col 3), rest centered (col 2).
const LAYOUT = {
  input: 2, road: 1, scene: 3, modeling: 3,
  xodr: 2, qa: 2, roadgraph: 2, xosc: 2, carla: 2, scene_outcome: 2, replay: 2,
};

// Edges drawn between nodes (source -> target).
const DAG_EDGES = [
  ["input", "road"], ["input", "scene"],
  ["road", "xodr"], ["scene", "modeling"], ["scene", "qa"],
  ["xodr", "qa"], ["qa", "roadgraph"],
  ["roadgraph", "xosc"], ["xosc", "carla"],
  ["carla", "scene_outcome"], ["scene_outcome", "replay"],
];

// Per-phase timing: phase_key -> [{round, t_start, t_end, duration_s, status}, ...]
const PHASE_TIMINGS = {};
function ingestTimings(arr) {
  STAGES.forEach(s => { PHASE_TIMINGS[s.key] = []; });
  (arr || []).forEach(entry => {
    if (!PHASE_TIMINGS[entry.phase]) PHASE_TIMINGS[entry.phase] = [];
    PHASE_TIMINGS[entry.phase].push(entry);
  });
}
ingestTimings((RESULT || {}).phase_timings);

function _statusClass(s) {
  if (s === "ok" || s === "pass") return "ok";
  if (s === "error" || s === "fail") return "bad";
  if (s === "running") return "running";
  if (s && (s.startsWith("skipped") || s === "warn" || s === "skipped")) return "warn";
  return "";
}

function renderNodes() {
  const nodesWrap = document.getElementById("svg-nodes");
  const edgesWrap = document.getElementById("svg-edges");
  if (!nodesWrap || !edgesWrap) return;
  // Edges first (under the nodes)
  edgesWrap.innerHTML = DAG_EDGES.map(([from, to]) => {
    const a = STAGES.find(s => s.key === from);
    const b = STAGES.find(s => s.key === to);
    if (!a || !b) return "";
    const stA = (NODE_STATE[from] || {}).status;
    const stB = (NODE_STATE[to] || {}).status;
    const done = (stA === "ok" || stA === "warn") && (stB === "ok" || stB === "warn" || stB === "running");
    const cls = done ? "svg-edge flow-done" : "svg-edge";
    const ax = a.x, ay = a.y + SVG_NH/2;
    const bx = b.x, by = b.y - SVG_NH/2 - 5;
    const midY = (ay + by) / 2;
    const d = `M${ax},${ay} C${ax},${midY} ${bx},${midY} ${bx},${by}`;
    const arrow = done ? "url(#arr-done)" : "url(#arr-idle)";
    return `<path class="${cls}" d="${d}" marker-end="${arrow}"/>`;
  }).join("");
  nodesWrap.innerHTML = STAGES.map((s, i) => {
    const st = NODE_STATE[s.key] || { status: "idle", note: "" };
    const timings = PHASE_TIMINGS[s.key] || [];
    const last = timings[timings.length - 1];
    const durStr = last && last.duration_s != null ? `${last.duration_s.toFixed(last.duration_s < 1 ? 2 : 1)}s` : "";
    const x = s.x - SVG_NW/2;
    const y = s.y - SVG_NH/2;
    const num = String(i + 1).padStart(2, "0");
    return `<g class="svg-node ${st.status} ${activeStage === s.key ? 'active' : ''}" data-stage="${s.key}" transform="translate(${x},${y})">
      <rect class="svg-node-bg" width="${SVG_NW}" height="${SVG_NH}" rx="10" ry="10"/>
      <text class="svg-node-num" x="10" y="15">${num}</text>
      <text class="svg-node-status" x="${SVG_NW - 10}" y="15" text-anchor="end">${st.status}</text>
      <text class="svg-node-title" x="10" y="34">${s.ico} ${s.title}</text>
      <text class="svg-node-dur" x="${SVG_NW - 10}" y="48" text-anchor="end">${durStr}</text>
    </g>`;
  }).join("");
  nodesWrap.querySelectorAll(".svg-node").forEach(n => {
    n.onclick = () => {
      activeStage = n.dataset.stage;
      nodesWrap.querySelectorAll(".svg-node").forEach(x => x.classList.toggle("active", x === n));
      const panel = document.getElementById("active-panel");
      if (panel) {
        renderDetail(activeStage, panel);
        panel.classList.remove("flash"); void panel.offsetWidth; panel.classList.add("flash");
      }
    };
  });
  renderGantt();
}

function drawDagEdges() {
  const svg = document.getElementById("dag-edges");
  const wrap = document.querySelector(".dag-wrap");
  const dag = document.getElementById("dag");
  if (!svg || !wrap || !dag) return;
  const wrapRect = wrap.getBoundingClientRect();
  svg.setAttribute("viewBox", `0 0 ${wrapRect.width} ${wrapRect.height}`);
  svg.setAttribute("width", wrapRect.width);
  svg.setAttribute("height", wrapRect.height);
  const nodeRects = {};
  dag.querySelectorAll(".node").forEach(n => {
    const r = n.getBoundingClientRect();
    nodeRects[n.dataset.stage] = {
      cx: r.left - wrapRect.left + r.width / 2,
      cy_top: r.top - wrapRect.top,
      cy_bot: r.bottom - wrapRect.top,
    };
  });
  const paths = DAG_EDGES.map(([from, to]) => {
    const a = nodeRects[from], b = nodeRects[to];
    if (!a || !b) return "";
    const x1 = a.cx, y1 = a.cy_bot;
    const x2 = b.cx, y2 = b.cy_top - 4;
    const dy = Math.max(20, (y2 - y1) * 0.5);
    return `<path d="M ${x1} ${y1} C ${x1} ${y1 + dy}, ${x2} ${y2 - dy}, ${x2} ${y2}" fill="none" stroke="#cdd6e6" stroke-width="1.6" marker-end="url(#arrow)"/>`;
  }).join("");
  document.getElementById("dag-edge-paths").innerHTML = paths;
}
window.addEventListener("resize", () => requestAnimationFrame(drawDagEdges));

function renderGantt() {
  const flat = [];
  STAGES.forEach((s, rowIdx) => (PHASE_TIMINGS[s.key] || []).forEach(t => {
    flat.push({ ...t, _row: rowIdx, _phase: s.key });
  }));
  const totalLabel = document.getElementById("gantt-total");
  if (totalLabel) {
    const _tMax = flat.length ? Math.max(...flat.map(f => f.t_end)) : 0;
    totalLabel.textContent = flat.length ? `共 ${_tMax.toFixed(1)}s` : "";
  }
  const svg = document.getElementById("gantt");
  if (!svg) return;
  if (!flat.length) {
    svg.innerHTML = `<text x="0" y="14" font-size="11" fill="#9aa4b5">尚无 phase 时长数据（仅 live 模式或 dispatch_full v2+ 才有）</text>`;
    svg.setAttribute("viewBox", `0 0 800 20`); svg.setAttribute("height", "20");
    return;
  }
  const tMax = Math.max(...flat.map(f => f.t_end));
  const tMin = 0;
  const rowH = 18, padTop = 6, padBot = 18, labelW = 86;
  const height = padTop + STAGES.length * rowH + padBot;
  const widthVB = 800;
  svg.setAttribute("viewBox", `0 0 ${widthVB} ${height}`);
  svg.setAttribute("height", height);
  const span = Math.max(1, tMax - tMin);
  const xScale = (t) => labelW + ((t - tMin) / span) * (widthVB - labelW - 20);
  // Row labels
  let out = "";
  STAGES.forEach((s, i) => {
    const y = padTop + i * rowH + rowH / 2;
    out += `<text class="gantt-row-label" x="6" y="${y + 3.5}">${s.ico} ${s.title}</text>`;
    out += `<line class="gantt-axis" x1="${labelW}" y1="${y - rowH/2 + 1}" x2="${widthVB - 10}" y2="${y - rowH/2 + 1}"/>`;
  });
  // Bars
  flat.forEach(f => {
    const y = padTop + f._row * rowH + 3;
    const x1 = xScale(f.t_start);
    const x2 = xScale(f.t_end);
    const w = Math.max(2, x2 - x1);
    const cls = _statusClass(f.status) || "ok";
    out += `<rect class="gantt-bar ${cls}" x="${x1}" y="${y}" width="${w}" height="${rowH - 6}">
      <title>${f._phase} · r${f.round} · ${f.status} · ${f.duration_s}s · [${f.t_start}s → ${f.t_end}s]</title>
    </rect>`;
    if (w > 36) {
      out += `<text x="${x1 + 4}" y="${y + rowH/2 + 1}" fill="#fff" font-size="9.5" font-family="ui-monospace,Menlo,monospace">r${f.round} · ${f.duration_s.toFixed(f.duration_s<1?2:1)}s</text>`;
    }
  });
  // X-axis ticks at 0, 25%, 50%, 75%, 100%.
  const baseY = padTop + STAGES.length * rowH + 2;
  out += `<line class="gantt-axis" x1="${labelW}" y1="${baseY}" x2="${widthVB - 10}" y2="${baseY}"/>`;
  for (let i = 0; i <= 4; i++) {
    const tt = tMin + (span * i / 4);
    const x = xScale(tt);
    out += `<line class="gantt-axis" x1="${x}" y1="${baseY}" x2="${x}" y2="${baseY + 4}"/>`;
    out += `<text class="gantt-axis-label" x="${x}" y="${baseY + 13}" text-anchor="middle">${tt.toFixed(1)}s</text>`;
  }
  svg.innerHTML = out;
  if (totalLabel) totalLabel.textContent = `total ${tMax.toFixed(1)}s · ${flat.length} phase events`;
}

function escapeHtml(s) { return String(s == null ? "" : s).replace(/[&<>]/g, c => ({"&":"&amp;","<":"&lt;",">":"&gt;"}[c])); }
function jsonBlock(o) { return `<pre class="code">${escapeHtml(JSON.stringify(o, null, 2))}</pre>`; }

function renderDetail(stage, box) {
  if (!stage || !box) return;
  const s = STAGES.find(x => x.key === stage);
  if (!s) return;
  const r = RESULT || {};
  const st = NODE_STATE[stage] || { status: "idle", note: "" };
  let body = "";

  if (stage === "input") {
    const t = r.extracted_text || "";
    body = `<div class="section"><div class="section-label">${r.input_kind === 'file' ? 'VLM 解读得到的事故文本' : '原始输入文本'} · ${r.text_chars||0} 字</div>${t ? `<pre class="text">${escapeHtml(t)}</pre>` : '<div class="empty">（无文本）</div>'}</div>`;
  } else if (stage === "road" || stage === "scene") {
    const seed = stage === "road" ? r.road_seed : r.scene_seed;
    if (!seed) body = `<div class="empty">尚无推理结果</div>`;
    else {
      body = `<div class="section"><div class="section-label">推理说明</div><div class="reason-box">${escapeHtml(seed.description_zh || seed.reason || '—')}</div></div>`;
      const normalizationRules = ((seed.pipeline || {}).normalization_rules || []);
      if (normalizationRules.length) {
        body += `<div class="section"><div class="section-label">时序规范化</div><div class="reason-box">${normalizationRules.map(escapeHtml).join(' · ')}</div></div>`;
      }
      body += `<div class="section"><div class="section-label">${stage} 结构</div>${jsonBlock(seed[stage] !== undefined ? seed[stage] : seed)}</div>`;
    }
  } else if (stage === "xodr") {
    if (r.xodr_path && r.html_path) {
      body = `<iframe class="preview" src="/run/${encodeURIComponent(RUN_NAME)}/xodr-preview?t=${Date.now()}"></iframe>`;
      if (r.summary) body += `<div class="section" style="margin-top:12px"><div class="section-label">道路统计</div><div class="metrics">` + Object.entries(r.summary).map(([k,v]) => v != null ? `<div class="metric"><div class="lbl">${k}</div><div class="val">${v}</div></div>` : '').join("") + `</div></div>`;
    } else {
      body = `<div class="empty">${escapeHtml(r.road_skipped || (r.errors && r.errors.compile) || '未生成 XODR')}</div>`;
    }
  } else if (stage === "qa") {
    const q = r.qa;
    if (!q || !q.rounds || !q.rounds.length) body = `<div class="empty">无 QA 记录</div>`;
    else {
      body = `<div class="section"><div class="section-label">最终评判</div><div class="reason-box"><b>${q.final_verdict}</b> · 跑了 ${q.rounds_run} / ${(q.max_retries||0)+1} 轮</div></div>`;
      q.rounds.forEach(round => {
        const composite = round.artifacts && round.artifacts.composite_png;
        body += `<div class="section" style="border-top:1px solid #eef1f6;padding-top:10px"><div class="section-label">第 ${round.round+1} 轮 · ${round.verdict||'skipped'}</div>`;
        if (composite) body += `<img class="qa-img" src="/run/${encodeURIComponent(RUN_NAME)}/qa-artifact/${round.round}/composite?t=${Date.now()}" />`;
        const issues = (round.issues || []).map(it => `<li><b>${escapeHtml(it.type||'?')}</b> · <span class="pill ${it.severity==='high'?'bad':(it.severity==='medium'?'warn':'')}">${escapeHtml(it.severity||'?')}</span> ${escapeHtml(it.description||'')}</li>`).join("");
        if (issues) body += `<div class="section"><div class="section-label">issues</div><ul style="margin:0;padding-left:18px;font-size:12.5px;line-height:1.6">${issues}</ul></div>`;
        if (round.fix_hint_road) body += `<div class="section"><div class="section-label">fix_hint → road</div><div class="reason-box">${escapeHtml(round.fix_hint_road)}</div></div>`;
        if (round.fix_hint_scene) body += `<div class="section"><div class="section-label">fix_hint → scene</div><div class="reason-box">${escapeHtml(round.fix_hint_scene)}</div></div>`;
        body += `</div>`;
      });
    }
  } else if (stage === "roadgraph") {
    const rg = r.roadgraph_status;
    if (rg === "ok") {
      const sc = r.roadgraph_selfcheck || {};
      body = `<div class="section"><div class="section-label">CARLA 道路图提取</div><div class="reason-box">把生成的 XODR 推到远端 CARLA，让它 load、抽 waypoint / junction / route。这一步也是对 XODR 几何的硬验证——CARLA 拒收的话 segfault 立刻报错，不会等到 carla_run 才暴露问题。</div></div>`;
      body += `<div class="metrics">`
        + metric("Continuity", sc.continuity_pass === true ? "pass" : (sc.continuity_pass === false ? "fail" : "—"), sc.continuity_pass === false ? "bad" : "ok")
        + metric("Waypoints", sc.waypoint_count != null ? sc.waypoint_count : '—')
        + metric("Junctions", sc.junction_count != null ? sc.junction_count : '—')
        + metric("Routes", sc.route_count != null ? sc.route_count : '—')
        + `</div>`;
      if (r.roadgraph_cache_dir) body += `<div class="section"><div class="section-label">Cache 路径</div><div class="reason-box"><code>${escapeHtml(r.roadgraph_cache_dir)}</code></div></div>`;
      body += `<div class="section"><div class="section-label">完整 selfcheck</div>${jsonBlock(sc)}</div>`;
    } else if (rg && rg.startsWith && rg.startsWith("error")) {
      body = `<div class="empty err">${escapeHtml(rg)}</div>`;
      const hint = (r.fix_loop_final_hints||{}).road;
      if (hint) body += `<div class="section"><div class="section-label">回灌给 road 的 fix_hint</div><div class="reason-box">${escapeHtml(hint)}</div></div>`;
    } else {
      body = `<div class="empty">${escapeHtml(rg || '未开始')}</div>`;
    }
  } else if (stage === "xosc") {
    if (r.xosc_status === "ok") {
      body = `<div class="section"><div class="section-label">编译结果</div><div class="reason-box">XOSC 已生成: <code>${escapeHtml(r.xosc_path||'')}</code></div></div>`;
      if (r.scene_seed && r.scene_seed.scene) body += `<div class="section"><div class="section-label">送入编译的 scene</div>${jsonBlock(r.scene_seed.scene)}</div>`;
    } else {
      body = `<div class="empty">${escapeHtml(r.xosc_status || '未编译')}</div>`;
    }
  } else if (stage === "carla") {
    if (r.carla_status === "ok" && r.carla_summary) {
      const cs = r.carla_summary;
      const so = r.scene_outcome || {};
      const collisionClass = so.aligned === true ? 'ok' : ((cs.collision_count||0) > 0 ? 'bad' : 'warn');
      body = `<div class="metrics">`
        + metric("ADS agent", r.pcla_agent || '—')
        + metric("Duration", cs.duration_seconds != null ? cs.duration_seconds.toFixed(1)+'s' : '—')
        + metric("Collisions", cs.collision_count != null ? cs.collision_count : '—', collisionClass)
        + metric("Lane invasions", cs.lane_invasion_count != null ? cs.lane_invasion_count : '—', (cs.lane_invasion_count||0) > 0 ? 'warn' : 'ok')
        + metric("min TTC", (cs.min_ttc||{}).value != null ? ((cs.min_ttc.value).toFixed(2)+'s') : '—', (cs.min_ttc||{}).value != null && cs.min_ttc.value < 2 ? 'bad' : '')
        + metric("min distance", (cs.min_distance||{}).value != null ? ((cs.min_distance.value).toFixed(2)+'m') : '—')
        + metric("Termination", cs.termination_reason || '—')
        + `</div>`;
      body += `<div class="section"><div class="section-label">完整 summary</div>${jsonBlock(cs)}</div>`;
    } else {
      body = `<div class="empty">${escapeHtml(r.carla_status || '尚未跑 CARLA')}</div>`;
    }
  } else if (stage === "scene_outcome") {
    const so = r.scene_outcome;
    if (!so) {
      body = `<div class="empty">尚无 L3 行为—意图判定</div>`;
    } else {
      const expected = (so.expected_collision || []).join(' ↔ ') || '—';
      const actual = (so.actual_collision_actors || []).join(' ↔ ') || '未检测到';
      const ready = (r.demo_readiness || {}).passed;
      body = `<div class="metrics">`
        + metric("Demo gate", ready === true ? 'PASS' : (ready === false ? 'FAIL' : '—'), ready === true ? 'ok' : (ready === false ? 'bad' : 'warn'))
        + metric("Verdict", so.verdict || '—', so.aligned === true ? 'ok' : (so.aligned === false ? 'bad' : 'warn'))
        + metric("Expected", expected)
        + metric("Actual", actual, so.aligned === true ? 'ok' : 'bad')
        + metric("Impact", so.impact_area || '—')
        + metric("Trajectory", so.trajectory_quality || '—', so.trajectory_quality === 'clean' ? 'ok' : (so.trajectory_quality === 'invalid' ? 'bad' : 'warn'))
        + metric("Lane invasions", so.lane_invasion_count != null ? so.lane_invasion_count : '—', (so.lane_invasion_count||0) > 0 ? 'warn' : 'ok')
        + metric("Lane-center overrun", so.off_road_time != null ? Number(so.off_road_time).toFixed(2)+'s' : '—', (so.off_road_time||0) > 0 ? 'warn' : 'ok')
        + metric("Max center excess", so.max_lateral_excess_m != null ? Number(so.max_lateral_excess_m).toFixed(3)+'m' : '—', (so.max_lateral_excess_m||0) > 0 ? 'warn' : 'ok')
        + metric("Before impact", so.off_road_before_collision_s != null ? Number(so.off_road_before_collision_s).toFixed(2)+'s' : '—')
        + metric("Wrong lane", so.wrong_lane_count != null ? so.wrong_lane_count : '—', (so.wrong_lane_count||0) > 0 ? 'bad' : 'ok')
        + `</div>`;
      if ((so.issues || []).length) {
        body += `<div class="section"><div class="section-label">判定问题</div><ul style="margin:0;padding-left:18px;line-height:1.7">${so.issues.map(x => `<li>${escapeHtml(x)}</li>`).join('')}</ul></div>`;
      } else {
        body += `<div class="section"><div class="section-label">结论</div><div class="reason-box">实际碰撞参与者与 seed 声明一致，且轨迹纪律未触发 L3 拒绝条件。</div></div>`;
      }
      if ((so.warnings || []).length) {
        body += `<div class="section"><div class="section-label">轨迹警告</div><ul style="margin:0;padding-left:18px;line-height:1.7">${so.warnings.map(x => `<li>${escapeHtml(x)}</li>`).join('')}</ul></div>`;
      }
      body += `<div class="section"><div class="section-label">完整 L3 记录</div>${jsonBlock(so)}</div>`;
    }
  } else if (stage === "replay") {
    body = renderReplay();
  } else if (stage === "modeling") {
    if (r.modeling_status === "ok" && r.scene_model_summary) {
      const sum = r.scene_model_summary;
      const types = (sum.relation_types || []).map(t => `<code>${escapeHtml(t)}</code>`).join(" · ");
      body = `
        <div class="section">
          <div class="section-label">场景元模型摘要 (M1)</div>
          <div class="reason-box">
            scenario type: <b>${escapeHtml(sum.scenario_type||'?')}</b> ·
            ego maneuver: ${escapeHtml(sum.sut_maneuver||'?')} ·
            control: ${escapeHtml(sum.control||'?')}<br/>
            <b>${sum.actor_count||0}</b> actors · <b>${sum.relation_count||0}</b> relations · <b>${sum.phase_count||0}</b> phases<br/>
            relation kinds: ${types || '—'}
          </div>
        </div>
        <div class="section">
          <div class="section-label">M1 实例图 — 本场景的 actors & relations</div>
          <div id="mm-instance" class="mermaid-host" style="background:var(--bg-elev); border:1px solid var(--bd); border-radius:10px; padding:14px; text-align:center; overflow-x:auto;">渲染中…</div>
        </div>
        <div class="section">
          <div class="section-label">行为活动图 — swimlane × phase × trigger</div>
          <div id="mm-activity" class="mermaid-host" style="background:var(--bg-elev); border:1px solid var(--bd); border-radius:10px; padding:14px; text-align:center; overflow-x:auto;">渲染中…</div>
        </div>
        <div class="section">
          <div class="section-label">M2 元模型类图 — Scene Metamodel</div>
          <div id="mm-class" class="mermaid-host" style="background:var(--bg-elev); border:1px solid var(--bd); border-radius:10px; padding:14px; text-align:center; overflow-x:auto;">渲染中…</div>
        </div>
        <div class="section">
          <div class="section-label">PlantUML 原文件（供论文导出 PNG/SVG）</div>
          <div class="reason-box">
            <a href="/run/${RUN_NAME}/modeling/metamodel_class.puml" target="_blank">metamodel_class.puml</a> ·
            <a href="/run/${RUN_NAME}/modeling/instance.puml" target="_blank">instance.puml</a> ·
            <a href="/run/${RUN_NAME}/modeling/activity.puml" target="_blank">activity.puml</a> ·
            <a href="/run/${RUN_NAME}/modeling/scene_model.json" target="_blank">scene_model.json</a>
          </div>
        </div>`;
    } else {
      body = `<div class="empty">${escapeHtml(r.modeling_status || '未生成')}</div>`;
    }
  }

  const stCls = _statusClass(st.status);
  const noteFrag = st.note ? `<span style="font-size:11px;color:var(--sub);margin-left:6px">· ${escapeHtml(st.note)}</span>` : '';
  box.innerHTML = `<div class="card-head"><span class="chev">▸</span><div class="node-ico">${s.ico}</div><h3>${s.title}${noteFrag}</h3><span class="badge ${stCls}">${st.status}</span></div><div class="card-body">${body}</div>`;
  if (stage === "replay") setupReplay();
  if (stage === "modeling") setupModeling();
}

function renderAllStages() {
  STAGES.forEach(s => {
    const box = document.getElementById('card-' + s.key);
    if (box) renderDetail(s.key, box);
  });
}

function metric(lbl, val, cls) { return `<div class="metric"><div class="lbl">${lbl}</div><div class="val ${cls||''}">${val}</div></div>`; }

let _mermaidReady = false;
function _ensureMermaid() {
  if (_mermaidReady) return true;
  if (typeof mermaid === "undefined") return false;
  mermaid.initialize({
    startOnLoad: false,
    theme: "dark",
    securityLevel: "loose",
    flowchart: { htmlLabels: true, curve: "basis" },
    themeVariables: {
      darkMode: true,
      background: "#0e1421",
      primaryColor: "#1a2138",
      primaryTextColor: "#e6e9f0",
      primaryBorderColor: "#2a3756",
      lineColor: "#8b95ab",
      secondaryColor: "#131826",
      tertiaryColor: "#0a0e1a",
      mainBkg: "#1a2138",
      textColor: "#e6e9f0",
      nodeBorder: "#2a3756",
      clusterBkg: "#0e1421",
      clusterBorder: "#2a3756",
      edgeLabelBackground: "#0e1421",
      tertiaryTextColor: "#e6e9f0",
    },
  });
  _mermaidReady = true;
  return true;
}
async function _renderMermaidInto(elId, mmdText) {
  const el = document.getElementById(elId);
  if (!el) return;
  try {
    const id = elId + "-svg-" + Date.now();
    const { svg } = await mermaid.render(id, mmdText);
    el.innerHTML = svg;
  } catch (e) {
    el.innerHTML = `<div class="empty err">mermaid 渲染失败: ${escapeHtml(String(e && e.message || e))}</div><pre class="code" style="text-align:left;">${escapeHtml(mmdText)}</pre>`;
  }
}
function setupModeling() {
  const tries = (n) => {
    if (_ensureMermaid()) {
      Promise.all([
        fetch(`/run/${RUN_NAME}/modeling/instance.mmd`).then(r => r.ok ? r.text() : Promise.reject(`HTTP ${r.status}`)),
        fetch(`/run/${RUN_NAME}/modeling/activity.mmd`).then(r => r.ok ? r.text() : Promise.reject(`HTTP ${r.status}`)),
        fetch(`/run/${RUN_NAME}/modeling/metamodel_class.mmd`).then(r => r.ok ? r.text() : Promise.reject(`HTTP ${r.status}`)),
      ]).then(([ins, act, cls]) => {
        _renderMermaidInto("mm-instance", ins);
        _renderMermaidInto("mm-activity", act);
        _renderMermaidInto("mm-class", cls);
      }).catch(err => {
        const a = document.getElementById("mm-activity");
        if (a) a.innerHTML = `<div class="empty err">无法获取 mermaid 源: ${escapeHtml(String(err))}</div>`;
      });
    } else if (n < 30) {
      setTimeout(() => tries(n + 1), 100);  // wait for CDN mermaid.js to load
    } else {
      const a = document.getElementById("mm-activity");
      if (a) a.innerHTML = `<div class="empty err">mermaid.js 加载超时（CDN 不可达？）</div>`;
    }
  };
  tries(0);
}

function renderReplay() {
  const r = RESULT || {};
  const so = r.scene_outcome || {};
  const hasVid = r.carla_status === "ok" && {{ 'true' if has_video else 'false' }};
  const hasMap = {{ 'true' if has_map else 'false' }};
  if (r.carla_status !== "ok") return `<div class="empty">CARLA 还没跑（或失败）。replay 不可用。</div>`;
  return `
    <div class="section"><div class="section-label">L3 行为—意图结果</div><div class="reason-box"><b>${escapeHtml(so.verdict || '未评判')}</b> · 预期 ${(so.expected_collision||[]).map(escapeHtml).join(' ↔ ') || '—'} · 实际 ${(so.actual_collision_actors||[]).map(escapeHtml).join(' ↔ ') || '—'}</div></div>
    <div class="replay-split">
      <div>${hasVid ? `<video id="vid" controls preload="metadata" src="/run/${encodeURIComponent(RUN_NAME)}/video"></video>` : '<div class="empty">无 mp4</div>'}</div>
      <div>
        <svg id="td" class="topdown" preserveAspectRatio="xMidYMid meet"></svg>
        <div class="clock" id="clk">t = 0.00s${hasMap ? '' : ' · 无地图底图'}</div>
      </div>
    </div>
    <div class="section" style="margin-top:14px"><div class="section-label">Events 时间轴（点击跳转视频）</div><ul id="ev-list" style="list-style:none;margin:0;padding:0;display:flex;flex-direction:column;gap:5px;max-height:200px;overflow:auto"></ul></div>
  `;
}

function setupReplay() {
  const svg = document.getElementById("td");
  const vid = document.getElementById("vid");
  const clk = document.getElementById("clk");
  if (!svg) return;
  const NS = "http://www.w3.org/2000/svg";
  const el = (tag, attrs) => { const n = document.createElementNS(NS, tag); for (const [k,v] of Object.entries(attrs||{})) n.setAttribute(k,v); return n; };
  const xs=[], ys=[];
  for (const f of RUN_DATA.frames) { xs.push(f.x); ys.push(f.y); }
  for (const r of RUN_DATA.roads) for (const [x,y] of r.points) { xs.push(x); ys.push(y); }
  if (!xs.length) { svg.innerHTML = '<text x="10" y="20" font-size="10" fill="#888">无 frames</text>'; return; }
  const pad=16, minX=Math.min(...xs)-pad, maxX=Math.max(...xs)+pad, minY=Math.min(...ys)-pad, maxY=Math.max(...ys)+pad;
  svg.setAttribute("viewBox", `${minX} ${-maxY} ${maxX-minX} ${maxY-minY}`);
  for (const road of RUN_DATA.roads) svg.appendChild(el("polyline", { class: "road-body", points: road.points.map(([x,y])=>`${x.toFixed(3)},${(-y).toFixed(3)}`).join(" ") }));
  if (RUN_DATA.frames.length) {
    svg.appendChild(el("polyline", { class: "traj", points: RUN_DATA.frames.map(f=>`${f.x.toFixed(3)},${(-f.y).toFixed(3)}`).join(" ") }));
    const s = RUN_DATA.frames[0], g = RUN_DATA.frames[RUN_DATA.frames.length-1];
    svg.appendChild(el("circle", { class:"start", cx:s.x, cy:-s.y, r:1.4 }));
    svg.appendChild(el("circle", { class:"goal",  cx:g.x, cy:-g.y, r:1.4 }));
  }
  for (const ev of RUN_DATA.events) {
    const p = poseAt(ev.t);
    if (p) svg.appendChild(el("circle", { class:"ev-dot "+ev.type, cx:p.x, cy:-p.y, r:1.2 }));
  }
  const f0 = RUN_DATA.frames[0];
  if (f0) svg.appendChild(el("circle", { class:"live", id:"liveDot", cx:f0.x, cy:-f0.y, r:1.7 }));

  const ul = document.getElementById("ev-list");
  if (ul) ul.innerHTML = RUN_DATA.events.map(ev =>
    `<li style="padding:6px 10px;background:#f8fafc;border:1px solid var(--bd);border-radius:6px;font-size:12px;display:flex;gap:8px;cursor:pointer">
      <span style="font-family:ui-monospace,Menlo,monospace;color:var(--sub);min-width:46px">${ev.t.toFixed(2)}s</span>
      <span style="font-weight:600;min-width:110px">${ev.type}</span>
      <span style="color:var(--sub);font-family:ui-monospace,Menlo,monospace;font-size:11px;overflow:hidden;text-overflow:ellipsis;white-space:nowrap;flex:1">${JSON.stringify(ev.payload||{})}</span>
    </li>`
  ).join("");
  if (ul && vid) Array.from(ul.children).forEach((li,i) => li.onclick = () => { vid.currentTime = RUN_DATA.events[i].t; updateLive(RUN_DATA.events[i].t); });

  function poseAt(t) {
    const F = RUN_DATA.frames;
    if (!F.length) return null;
    if (t <= F[0].t) return F[0];
    if (t >= F[F.length-1].t) return F[F.length-1];
    let lo=0, hi=F.length-1;
    while (hi-lo > 1) { const mid = (lo+hi)>>1; if (F[mid].t <= t) lo = mid; else hi = mid; }
    const a=F[lo], b=F[hi], u=(t-a.t)/Math.max(1e-6, b.t-a.t);
    return { x:a.x+(b.x-a.x)*u, y:a.y+(b.y-a.y)*u };
  }
  function updateLive(t) {
    const p = poseAt(t), dot = document.getElementById("liveDot");
    if (!p || !dot) return;
    dot.setAttribute("cx", p.x.toFixed(3)); dot.setAttribute("cy", (-p.y).toFixed(3));
    clk.textContent = `t = ${t.toFixed(2)}s`;
  }
  if (vid) {
    vid.addEventListener("timeupdate", () => updateLive(vid.currentTime));
    vid.addEventListener("seeked",     () => updateLive(vid.currentTime));
  }
}

// Helper: render the currently selected stage into the right-side single panel.
function renderActivePanel() {
  const panel = document.getElementById("active-panel");
  if (!panel) return;
  if (!activeStage) return;
  renderDetail(activeStage, panel);
}

activeStage = JOB_ACTIVE ? "input" : ((RESULT || {}).scene_outcome ? "scene_outcome" : "input");
deriveStatesFromResult();
renderNodes();
renderActivePanel();

if (JOB_ACTIVE) {
  // Live mode: stream phase events and refresh node states.
  const es = new EventSource(`/pipeline/${encodeURIComponent(RUN_NAME)}/stream`);
  function setStatus(phase, status, note) {
    NODE_STATE[phase] = { status, note: note || (NODE_STATE[phase] || {}).note || "" };
    renderNodes();
    if (activeStage === phase) renderActivePanel();
  }
  function ingestLivePhase(p, status) {
    if (!p || !p.phase) return;
    if (!PHASE_TIMINGS[p.phase]) PHASE_TIMINGS[p.phase] = [];
    const round = p.round != null ? p.round : -1;
    const entry = {
      phase: p.phase, round,
      t_start: p.t_start, t_end: p.t_end,
      duration_s: p.duration_s, status,
    };
    // Replace existing entry for same (phase, round) so retries update in place.
    const i = PHASE_TIMINGS[p.phase].findIndex(t => t.round === round);
    if (i >= 0) PHASE_TIMINGS[p.phase][i] = entry; else PHASE_TIMINGS[p.phase].push(entry);
  }
  es.addEventListener("phase_start", e => {
    const p = JSON.parse(e.data); setStatus(p.phase, "running", "运行中…");
  });
  es.addEventListener("phase_retry", e => {
    const p = JSON.parse(e.data);
    const banner = document.querySelector(".live-banner");
    if (banner) banner.innerHTML = `<div class="live-dot"></div><span>第 ${p.round + 1} 轮 · 原因：${p.reason || 'fix_hint'}</span>`;
    // Reset upstream nodes to "running" since we're re-rolling them.
    ["road", "scene", "xodr", "qa"].forEach(k => setStatus(k, "running", `重跑 r${p.round}`));
  });
  es.addEventListener("phase_done", e => {
    const p = JSON.parse(e.data);
    const note = p.status || p.verdict || (p.duration_s ? `${p.duration_s}s` : "ok");
    let s = "ok";
    if (p.status && p.status.startsWith && p.status.startsWith("skipped")) s = "warn";
    if (p.verdict === "fail" || p.verdict === "drifted") s = "bad";
    if (p.verdict === "skipped") s = "warn";
    ingestLivePhase(p, s === "ok" ? (p.verdict || "ok") : (p.status || p.verdict || s));
    setStatus(p.phase, s, note);
  });
  es.addEventListener("phase_error", e => {
    const p = JSON.parse(e.data);
    ingestLivePhase(p, "error");
    setStatus(p.phase, "bad", p.error || "error");
  });
  es.addEventListener("done", e => {
    try { RESULT = JSON.parse(e.data); } catch {}
    deriveStatesFromResult(); renderNodes(); renderActivePanel();
    es.close();
    // Quick refresh so on-disk artifacts (frames.json, video) are picked up.
    setTimeout(() => location.replace(location.pathname), 800);
  });
  es.addEventListener("fatal", e => {
    const p = JSON.parse(e.data); alert("流水线出错: " + (p.error || ""));
    es.close();
  });
}
</script>
</body></html>
""".replace("__STYLE__", _STYLE_BASE)


# ---- CLI --------------------------------------------------------------------


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Unified dashboard for the road-scene pipeline")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5000)
    ap.add_argument("--out-root", type=Path, default=DEFAULT_OUT_ROOT)
    ap.add_argument("--debug", action="store_true")
    return ap.parse_args()


def main() -> int:
    args = parse_args()
    out_root = args.out_root if args.out_root.is_absolute() else REPO_ROOT / args.out_root
    CONFIG["out_root"] = out_root
    out_root.mkdir(parents=True, exist_ok=True)

    # Load .env.local once so OPENROUTER_API_KEY is available to background jobs.
    try:
        from tools.coordinator import load_env
        load_env(DEFAULT_ENV)
    except Exception:
        pass

    runs = _scan_runs()
    missing_map = [r["name"] for r in runs if not r["has_map"]]
    print(f"[web] out_root={out_root}  runs={len(runs)}")
    if missing_map:
        print(f"[web] {len(missing_map)} run(s) without map.xodr will render trajectories only:")
        for n in missing_map:
            print(f"  - {n}")
    if "OPENROUTER_API_KEY" not in __import__("os").environ:
        print("[web] WARN: OPENROUTER_API_KEY not in env; /new pipelines will fail at inference",
              file=sys.stderr)
    print(f"[web] serving on http://{args.host}:{args.port}  (threaded=True for SSE)")
    app.run(host=args.host, port=args.port, debug=args.debug, threaded=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
