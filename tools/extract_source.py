#!/usr/bin/env python3
"""Input normalization: any supported source -> raw text blob for the agents.

PDF goes through a VLM (deepseek-flash by default) that reads every page as an
image and emits a faithful event-description text. Other formats stay text-only.

    .pdf                          -> VLM (page images -> description blob)
    .docx/.pptx/.xlsx/.html/.csv  -> markitdown
    .txt/.md                      -> read as-is
    a raw paragraph (non-path str)-> returned unchanged

    uv run --with pymupdf --with openai --with "markitdown[pdf]" \
        python tools/extract_source.py <path-or-text>
"""

from __future__ import annotations

import base64
import difflib
import hashlib
import json
import os
import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

MARKITDOWN_SUFFIXES = {".docx", ".pptx", ".xlsx", ".html", ".htm", ".csv"}
TEXT_SUFFIXES = {".txt", ".md"}
KNOWN_SUFFIXES = {".pdf"} | MARKITDOWN_SUFFIXES | TEXT_SUFFIXES


def pdf_narrative_coverage(source: Path, text: str) -> dict:
    """Detect lost long form fields, especially DMV's accident description.

    This verifies the VLM output against the same original PDF; field text is
    never substituted for model output. Scans without form fields remain subject
    to image review and cannot pass this particular completeness check by proxy.
    """
    if source.suffix.lower() != ".pdf":
        return {"applicable": False, "passed": True, "fields": []}
    import fitz
    fields = []
    output = re.findall(r"[a-z0-9]+", text.lower())
    with fitz.open(source) as document:
        for page_number, page in enumerate(document, 1):
            for widget in page.widgets() or []:
                value = str(widget.field_value or "").strip()
                if len(value) < 200:
                    continue
                words = re.findall(r"[a-z0-9]+", value.lower())
                if not words:
                    continue
                matched = sum(b.size for b in difflib.SequenceMatcher(
                    None, words, output, autojunk=False).get_matching_blocks())
                fields.append({"page": page_number, "field": widget.field_name,
                               "word_count": len(words), "coverage": round(matched / len(words), 4)})
    return {"applicable": bool(fields), "passed": all(f["coverage"] >= .85 for f in fields),
            "fields": fields}


def load_extraction_checkpoint(source: Path, checkpoint: Path, *, model: str, base_url: str):
    """Resume a recorded PDF read only when its source and text are unchanged."""
    record = json.loads((checkpoint / "source_extraction.json").read_text())
    payload = (checkpoint / "source_text.txt").read_bytes()
    expected = {"source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
                "text_sha256": hashlib.sha256(payload).hexdigest(),
                "vlm_model": model, "base_url": base_url}
    for key, value in expected.items():
        if record.get(key) != value:
            raise ValueError(f"Source extraction checkpoint mismatch: {key}")
    text = payload.decode("utf-8")
    if not text.strip() or not record.get("extracted_at"):
        raise ValueError("Incomplete source extraction checkpoint")
    if not pdf_narrative_coverage(source, text)["passed"]:
        raise ValueError("Source extraction checkpoint omits PDF narrative fields")
    return text, record


VLM_SYSTEM_PROMPT = (
    "你是事故报告解读器。用户会一次性给你一份事故 PDF 的所有页面图像。"
    "请把所有可读事实——事故经过叙述、时间地点、参与者(人/车/类型/品牌型号/控制模式)、"
    "环境(天气/光照/路面)、道路与交通控制(车道、信号、标志)、车辆动作序列、"
    "碰撞类型与冲击位置、损伤——按原报告组织顺序汇总成一段忠实文本。"
    "原文是英文就保留英文;表单字段、人名、地名、街道、车辆编号原样保留;"
    "不要总结、不要省略、不要做事实之外的推断、不要加判断或建议。"
    "若某页是事故几何图示,简要客观描述该图所示的道路布局或车辆相对位置即可。"
    "只输出文本本身,不要 JSON、不要 markdown 代码块、不要任何前后缀。"
)


def _looks_like_path(value: str) -> bool:
    try:
        return Path(value).exists()
    except OSError:  # e.g. a long paragraph triggers ENAMETOOLONG
        return False


def _render_pdf_pages_b64(path: Path, dpi: int = 150) -> list[str]:
    """Render every PDF page as a base64-encoded PNG."""
    import fitz

    # Some valid interactive DMV forms have a malformed accessibility
    # structure tree. MuPDF can still render every page, but otherwise prints
    # alarming non-fatal diagnostics directly to stderr during a live demo.
    show_errors = fitz.TOOLS.mupdf_display_errors()
    show_warnings = fitz.TOOLS.mupdf_display_warnings()
    fitz.TOOLS.mupdf_display_errors(False)
    fitz.TOOLS.mupdf_display_warnings(False)
    doc = None
    try:
        doc = fitz.open(str(path))
        pages: list[str] = []
        for page in doc:
            pix = page.get_pixmap(dpi=dpi)
            pages.append(base64.b64encode(pix.tobytes("png")).decode("ascii"))
        return pages
    finally:
        if doc is not None:
            doc.close()
        fitz.TOOLS.mupdf_display_errors(show_errors)
        fitz.TOOLS.mupdf_display_warnings(show_warnings)


def _extract_pdf_vlm(
    path: Path,
    *,
    model: str,
    base_url: str,
    api_key_env: str,
    dpi: int = 150,
    max_tokens: int = 8000,
) -> str:
    """Send every page image to the VLM and return the model's plain-text recap.

    DMV OL316 reports mix narrative text, interactive form fields, and small
    accident-geometry diagrams. A VLM that sees the full page image picks up all
    three at once — no separate widget reader or diagram parser is needed.
    """
    from openai import OpenAI

    api_key = os.environ.get(api_key_env)
    if not api_key:
        raise RuntimeError(f"Missing {api_key_env} for VLM PDF extraction")
    page_b64s = _render_pdf_pages_b64(path, dpi=dpi)
    if not page_b64s:
        return ""
    user_content: list[dict] = [
        {"type": "text",
         "text": f"以下是一份 {len(page_b64s)} 页事故报告 PDF 的全部页面图像。请按系统提示要求,输出完整事实文本。"},
    ]
    for b64 in page_b64s:
        user_content.append({"type": "image_url",
                             "image_url": {"url": f"data:image/png;base64,{b64}"}})
    client = OpenAI(api_key=api_key, base_url=base_url)
    # Reasoning VLMs occasionally burn the whole token budget thinking and
    # return an empty content string; providers also hiccup. Retry (with a
    # bigger budget) instead of silently handing "" to the seed agents.
    finish = None
    coverage = None
    for attempt in range(3):
        from tools.model_transport import chat_completion
        resp = chat_completion(client,
            model=model,
            stream=False,
            temperature=0.0,
            max_tokens=max_tokens * (2 ** attempt),
            messages=[
                {"role": "system", "content": VLM_SYSTEM_PROMPT},
                {"role": "user", "content": user_content},
            ],
        )
        text = (resp.choices[0].message.content or "").strip()
        finish = resp.choices[0].finish_reason
        coverage = pdf_narrative_coverage(path, text)
        if text and finish != "length" and coverage["passed"]:
            return text
        if text:
            print(f"[extract] Incomplete PDF read (finish={finish}, narrative={coverage}); retrying", flush=True)
            if attempt == 0:
                user_content.insert(0, {"type": "text", "text":
                    "上次读取遗漏或截断了内容。请优先完整逐字转写 SECTION 5 的事故经过，"
                    "再输出其他各页事实，务必读完每页底部与所有后续页面，不要只转写前半部分表单。"})
    raise RuntimeError(
        f"VLM PDF extraction returned incomplete text after 3 attempts "
        f"(model={model}, last finish_reason={finish}, coverage={coverage})"
    )


def _extract_markitdown(path: Path) -> str:
    from markitdown import MarkItDown

    return MarkItDown().convert(str(path)).text_content


def extract_any(
    source: str | Path,
    *,
    vlm_model: str | None = None,
    vlm_base_url: str | None = None,
    vlm_api_key_env: str | None = None,
) -> str:
    """Normalize a file path or a raw paragraph into a plain-text blob.

    PDF inputs go through the VLM when ``vlm_model`` is provided (and the
    matching api-key env var is set). All other inputs are text-only.
    """
    if isinstance(source, str) and not _looks_like_path(source):
        if Path(source).suffix.lower() in KNOWN_SUFFIXES or os.sep in source:
            raise FileNotFoundError(f"source looks like a path but does not exist: {source}")
        return source

    path = Path(source)
    if not path.exists():
        raise FileNotFoundError(f"source not found: {path}")

    suffix = path.suffix.lower()
    if suffix == ".pdf":
        from tools.model_transport import model_settings
        defaults = model_settings()
        return _extract_pdf_vlm(
            path,
            model=vlm_model or defaults["vlm_model"],
            base_url=vlm_base_url or defaults["base_url"],
            api_key_env=vlm_api_key_env or defaults["api_key_env"],
        )
    if suffix in MARKITDOWN_SUFFIXES:
        return _extract_markitdown(path)
    if suffix in TEXT_SUFFIXES:
        return path.read_text(encoding="utf-8")
    raise ValueError(f"unsupported source type {suffix!r}: {path}")


def main() -> int:
    if len(sys.argv) < 2:
        print("usage: extract_source.py <path-or-text>", file=sys.stderr)
        return 2
    # Best-effort .env load so the CLI works the same way as coordinator.py.
    env_path = Path(__file__).resolve().parents[1] / ".env.local"
    if env_path.exists():
        for line in env_path.read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            k, _, v = line.partition("=")
            os.environ.setdefault(k.strip(), v.strip().strip('"').strip("'"))
    text = extract_any(sys.argv[1])
    print(f"[extract] chars={len(text)}")
    print("=" * 40)
    print(text[:2000])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
