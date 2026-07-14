#!/usr/bin/env python3
"""Input normalization: any supported source -> raw text blob for the agents.

PDF goes through a VLM (xiaomi/mimo-v2.5 by default) that reads every page as an
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
import os
import sys
from pathlib import Path

MARKITDOWN_SUFFIXES = {".docx", ".pptx", ".xlsx", ".html", ".htm", ".csv"}
TEXT_SUFFIXES = {".txt", ".md"}
KNOWN_SUFFIXES = {".pdf"} | MARKITDOWN_SUFFIXES | TEXT_SUFFIXES


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
    resp = client.chat.completions.create(
        model=model,
        stream=False,
        temperature=0.0,
        max_tokens=max_tokens,
        messages=[
            {"role": "system", "content": VLM_SYSTEM_PROMPT},
            {"role": "user", "content": user_content},
        ],
    )
    return (resp.choices[0].message.content or "").strip()


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
        if not vlm_model:
            raise RuntimeError(
                "PDF extraction now requires a VLM. Pass vlm_model/vlm_base_url/vlm_api_key_env "
                "(or set VLM_MODEL/OPENROUTER_BASE_URL/OPENROUTER_API_KEY in .env.local)."
            )
        return _extract_pdf_vlm(
            path,
            model=vlm_model,
            base_url=vlm_base_url or "https://openrouter.ai/api/v1",
            api_key_env=vlm_api_key_env or "OPENROUTER_API_KEY",
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
    text = extract_any(
        sys.argv[1],
        vlm_model=os.environ.get("VLM_MODEL"),
        vlm_base_url=os.environ.get("OPENROUTER_BASE_URL"),
        vlm_api_key_env="OPENROUTER_API_KEY",
    )
    print(f"[extract] chars={len(text)}")
    print("=" * 40)
    print(text[:2000])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
