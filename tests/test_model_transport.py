import threading
import time
from concurrent.futures import ThreadPoolExecutor
from types import SimpleNamespace

import httpx
import pytest
from openai import APIStatusError

from tools import model_transport as transport


@pytest.mark.parametrize("message,expected", [
    ("402 openrouter_credits requires more credits", "model_credits"),
    ("Error code: 402 - Insufficient Balance", "model_credits"),
    ("402 in_flight_budget_exhausted", "model_inflight_budget"),
    ("JSONDecodeError: invalid model output", None)])
def test_shared_resource_failures_stop_the_batch(message, expected):
    assert transport.model_resource_blocker(message) == expected


def client_with(create):
    return SimpleNamespace(base_url="https://test.invalid/v1", chat=SimpleNamespace(
        completions=SimpleNamespace(create=create)))


def test_model_requests_share_endpoint_slot(monkeypatch, tmp_path):
    monkeypatch.setenv("C2X_SERIALIZE_MODEL_CALLS", "1")
    monkeypatch.setattr(transport.tempfile, "gettempdir", lambda: str(tmp_path))
    active = 0
    maximum = 0
    mutex = threading.Lock()

    def create(**kwargs):
        nonlocal active, maximum
        with mutex:
            active += 1
            maximum = max(maximum, active)
        time.sleep(0.03)
        with mutex:
            active -= 1
        return kwargs["model"]

    with ThreadPoolExecutor(2) as pool:
        calls = [pool.submit(transport.chat_completion, client_with(create), model=m)
                 for m in ("road", "scene")]
        assert [f.result() for f in calls] == ["road", "scene"]
    assert maximum == 1


@pytest.mark.parametrize("reason,expected_calls", [
    ("in_flight_budget_exhausted", 2), ("insufficient_credits", 1)])
def test_retry_only_inflight_budget(monkeypatch, reason, expected_calls):
    monkeypatch.delenv("C2X_SERIALIZE_MODEL_CALLS", raising=False)
    now = [0.0]
    monkeypatch.setattr(transport.time, "monotonic", lambda: now[0])
    monkeypatch.setattr(transport.time, "sleep", lambda seconds: now.__setitem__(0, now[0] + seconds))
    response = httpx.Response(402, headers={"Retry-After": "120"},
                              request=httpx.Request("POST", "https://test.invalid/v1"))
    calls = []

    def create(**kwargs):
        calls.append(kwargs)
        if len(calls) == 1:
            raise APIStatusError(reason, response=response, body={"metadata": {"reason": reason}})
        return "success"

    if expected_calls == 2:
        assert transport.chat_completion(client_with(create), model="test") == "success"
        assert now[0] == 120
    else:
        with pytest.raises(APIStatusError):
            transport.chat_completion(client_with(create), model="test")
        assert now[0] == 0
    assert len(calls) == expected_calls


def test_all_model_roles_resolve_new_provider_without_secret_values(monkeypatch):
    monkeypatch.setenv("TEXT_MODEL", "deepseek-flash")
    monkeypatch.setenv("VLM_MODEL", "deepseek-flash")
    monkeypatch.setenv("MODEL_BASE_URL", "https://api.deepseek.com")
    monkeypatch.setenv("MODEL_API_KEY_ENV", "DEEPSEEK_API_KEY")
    monkeypatch.setenv("OPENROUTER_BASE_URL", "https://old.invalid")
    monkeypatch.setenv("DEEPSEEK_API_KEY", "test-secret-never-returned")
    assert transport.model_settings() == {
        "model": "deepseek-flash", "vlm_model": "deepseek-flash",
        "base_url": "https://api.deepseek.com", "api_key_env": "DEEPSEEK_API_KEY"}


def test_fresh_process_defaults_to_native_deepseek_despite_legacy_provider_env(monkeypatch):
    for key in ('TEXT_MODEL', 'VLM_MODEL', 'MODEL_BASE_URL', 'MODEL_API_KEY_ENV'):
        monkeypatch.delenv(key, raising=False)
    monkeypatch.setenv('OPENROUTER_BASE_URL', 'https://old.invalid')
    assert transport.model_settings() == {
        'model': 'deepseek-flash', 'vlm_model': 'deepseek-flash',
        'base_url': 'https://api.deepseek.com', 'api_key_env': 'DEEPSEEK_API_KEY'}


def test_native_deepseek_image_and_qa_calls_receive_thinking_options(monkeypatch):
    monkeypatch.setenv("MODEL_ENABLE_THINKING", "1")
    monkeypatch.setenv("MODEL_REASONING_EFFORT", "high")
    monkeypatch.delenv("C2X_SERIALIZE_MODEL_CALLS", raising=False)
    client = client_with(lambda **kwargs: kwargs)
    client.base_url = "https://api.deepseek.com/"
    result = transport.chat_completion(client, model="deepseek-flash", extra_body={"other": True})
    assert result['reasoning_effort'] == 'high'
    assert result['extra_body'] == {'other': True, 'thinking': {'type': 'enabled'}}
    # Explicit per-call settings remain authoritative.
    result = transport.chat_completion(client, model="deepseek-flash", reasoning_effort="low",
                                       extra_body={"thinking": {"type": "disabled"}})
    assert result['reasoning_effort'] == 'low'
    assert result['extra_body']['thinking']['type'] == 'disabled'


@pytest.mark.parametrize('content,finish', [('', 'stop'), ('{"status":"supported"}', 'length')])
def test_seed_completion_retries_empty_or_truncated_output(monkeypatch, content, finish):
    from types import SimpleNamespace
    requests = []
    responses = iter([(content, finish), ('{"status":"supported"}', 'stop')])
    def complete(client, **kwargs):
        requests.append(kwargs)
        text, reason = next(responses)
        return SimpleNamespace(choices=[SimpleNamespace(
            message=SimpleNamespace(content=text), finish_reason=reason)])
    monkeypatch.setattr(transport, 'chat_completion', complete)
    result = transport.complete_chat_completion(object(), model='test', max_tokens=8000)
    assert result.choices[0].finish_reason == 'stop'
    assert [r['max_tokens'] for r in requests] == [8000, 16000]


def test_seed_completion_reports_budget_failure_instead_of_parsing_empty_json(monkeypatch):
    from types import SimpleNamespace
    monkeypatch.setattr(transport, 'chat_completion', lambda *args, **kwargs:
        SimpleNamespace(choices=[SimpleNamespace(message=SimpleNamespace(content=''),
                                                finish_reason='length')]))
    with pytest.raises(RuntimeError, match='incomplete seed response after 3 attempts'):
        transport.complete_chat_completion(object(), model='test')


@pytest.mark.parametrize('requested,expected', [(64, [8000, 16000, 32000]), (12000, [12000, 24000, 32000])])
def test_native_thinking_budget_has_headroom_and_bounded_retries(monkeypatch, requested, expected):
    calls = []
    def complete(client, **kwargs):
        calls.append(kwargs['max_tokens'])
        return SimpleNamespace(choices=[SimpleNamespace(message=SimpleNamespace(content=''), finish_reason='length')])
    monkeypatch.setattr(transport, 'chat_completion', complete)
    client = SimpleNamespace(base_url='https://api.deepseek.com')
    with pytest.raises(RuntimeError, match='incomplete seed response'):
        transport.complete_chat_completion(client, model='deepseek-flash', max_tokens=requested)
    assert calls == expected
