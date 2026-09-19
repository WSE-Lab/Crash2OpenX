"""Optional cross-process API serialization for small in-flight budgets."""
import hashlib
import os
import tempfile
import time
from contextlib import contextmanager
from pathlib import Path
from urllib.parse import urlparse


def model_settings():
    """Resolve the shared text, vision and QA provider without reading secrets."""
    return {
        "model": os.environ.get("TEXT_MODEL", "deepseek-flash"),
        "vlm_model": os.environ.get("VLM_MODEL", "deepseek-flash"),
        "base_url": os.environ.get("MODEL_BASE_URL", "https://api.deepseek.com"),
        "api_key_env": os.environ.get("MODEL_API_KEY_ENV", "DEEPSEEK_API_KEY"),
    }


def model_resource_blocker(error) -> str | None:
    message = str(error)
    if "402" not in message:
        return None
    if "in_flight_budget_exhausted" in message:
        return "model_inflight_budget"
    if ("openrouter_credits" in message or "requires more credits" in message
            or "insufficient_credits" in message or "insufficient balance" in message.lower()):
        return "model_credits"
    return None


@contextmanager
def _request_slot(client):
    if os.environ.get("C2X_SERIALIZE_MODEL_CALLS") != "1":
        yield
        return
    import fcntl
    identity = str(client.base_url).encode()
    path = Path(tempfile.gettempdir()) / ("c2x_model_" + hashlib.sha256(identity).hexdigest()[:16] + ".lock")
    with path.open("a") as handle:
        fcntl.flock(handle, fcntl.LOCK_EX)
        try:
            yield
        finally:
            fcntl.flock(handle, fcntl.LOCK_UN)


def chat_completion(client, **kwargs):
    from openai import APIStatusError
    if urlparse(str(client.base_url)).hostname == "api.deepseek.com":
        kwargs.setdefault("reasoning_effort", os.environ.get("MODEL_REASONING_EFFORT", "high"))
        if os.environ.get("MODEL_ENABLE_THINKING", "1") == "1":
            extra = dict(kwargs.get("extra_body") or {})
            extra.setdefault("thinking", {"type": "enabled"})
            kwargs["extra_body"] = extra
    with _request_slot(client):
        for attempt in range(4):
            try:
                return client.chat.completions.create(**kwargs)
            except APIStatusError as exc:
                if (exc.status_code != 402 or "in_flight_budget_exhausted" not in str(exc)
                        or attempt == 3):
                    raise
                try:
                    delay = float(exc.response.headers.get("Retry-After", "120"))
                except ValueError:
                    delay = 120
                delay = min(300, max(1, delay))
                print(f"Model API in-flight budget busy; retry {attempt + 1}/3 after {delay:g}s", flush=True)
                until = time.monotonic() + delay
                while time.monotonic() < until:
                    time.sleep(min(60, max(0, until - time.monotonic())))


def complete_chat_completion(client, **kwargs):
    """Require nonempty, untruncated content before parsing a seed response.

    Thinking tokens share the completion budget on the native provider. A
    length-limited answer is not evidence of an unsupported source scenario.
    """
    budget = kwargs.pop("max_tokens", 8000)
    native = urlparse(str(getattr(client, "base_url", ""))).hostname == "api.deepseek.com"
    if native:
        budget = max(8000, min(32000, budget))
    for attempt in range(3):
        output_budget = budget * (2 ** attempt)
        if native:
            output_budget = min(32000, output_budget)
        response = chat_completion(client, max_tokens=output_budget, **kwargs)
        choice = response.choices[0]
        content = choice.message.content or ""
        if content.strip() and choice.finish_reason != "length":
            return response
        if attempt < 2:
            print(f"[model] Incomplete seed response (finish={choice.finish_reason}, "
                  f"chars={len(content)}); retrying with larger output budget", flush=True)
    raise RuntimeError(
        f"Model returned incomplete seed response after 3 attempts "
        f"(model={kwargs.get('model')}, finish_reason={choice.finish_reason}, chars={len(content)})")
