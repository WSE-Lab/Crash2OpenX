"""Shared helpers for selecting real Chinese accident scene descriptions."""

from __future__ import annotations

from typing import Any


PLACEHOLDER_MARKERS = (
    "根据报告叙述",
    "具体位置、参与方和损伤情况以英文原始叙述为准",
    "以英文原始叙述为准",
)


def is_placeholder_chinese_scene_description(text: Any) -> bool:
    value = str(text or "").strip()
    return any(marker in value for marker in PLACEHOLDER_MARKERS)


def looks_like_chinese_summary(text: Any) -> bool:
    value = str(text or "").strip()
    cjk = sum(1 for char in value if "\u4e00" <= char <= "\u9fff")
    latin = sum(1 for char in value if ("a" <= char.lower() <= "z"))
    return cjk >= 12 and cjk / max(1, cjk + latin) >= 0.35


def valid_chinese_scene_description(text: Any) -> bool:
    value = str(text or "").strip()
    if not value or value.startswith(("{", "[")):
        return False
    if "event_description_text" in value or "'metadata'" in value or '"metadata"' in value:
        return False
    return looks_like_chinese_summary(value) and not is_placeholder_chinese_scene_description(value)
