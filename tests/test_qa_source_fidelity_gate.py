import pytest
from types import SimpleNamespace

from tools.qa_agent import _normalize_verdict


@pytest.mark.parametrize('severity', ['medium', 'high', 'unknown', None])
def test_behavior_issue_cannot_be_overridden_by_model_pass(severity):
    report = {'verdict': 'pass', 'issues': [{'severity': severity,
              'type': 'scene_correctness', 'description': 'A stopped lead was incorrectly made to brake.'}],
              'fix_hint_scene': 'Use the source-described stationary state.'}
    result = _normalize_verdict(report)
    assert result['verdict'] == 'fail'
    assert result['fix_hint_scene'] == report['fix_hint_scene']


def test_low_only_comments_can_still_pass():
    assert _normalize_verdict({'verdict': 'pass', 'issues': [{'severity': 'low'}]})['verdict'] == 'pass'
    assert _normalize_verdict({'verdict': 'fail', 'issues': []})['verdict'] == 'fail'


@pytest.mark.parametrize('first_content', ['', '{"verdict":"pass","issues":[]}'])
def test_qa_retries_truncated_answer_and_preserves_complete_source(monkeypatch, first_content):
    import openai
    from tools import model_transport, qa_agent

    requests = []
    responses = iter([(first_content, 'length'), ('{"verdict":"pass","issues":[]}', 'stop')])

    def complete(client, **kwargs):
        requests.append(kwargs)
        content, finish = next(responses)
        return SimpleNamespace(choices=[SimpleNamespace(
            message=SimpleNamespace(content=content), finish_reason=finish)])

    monkeypatch.setenv('QA_TEST_KEY', 'test-key')
    monkeypatch.setattr(openai, 'OpenAI', lambda **kwargs: object())
    monkeypatch.setattr(model_transport, 'chat_completion', complete)
    source = 'A' * 7000 + ' The following vehicle struck the rear.'
    result = qa_agent.call_qa_vlm(
        composite_png=b'png', event_text=source, road_seed={}, scene_seed={},
        vlm_model='test-model', base_url='https://test.invalid', api_key_env='QA_TEST_KEY')

    assert [r['max_tokens'] for r in requests] == [8000, 16000]
    assert source in requests[0]['messages'][1]['content'][0]['text']
    assert result['verdict'] == 'pass'


def test_qa_never_accepts_a_truncated_final_verdict(monkeypatch):
    import openai
    from tools import model_transport, qa_agent

    monkeypatch.setenv('QA_TEST_KEY', 'test-key')
    monkeypatch.setattr(openai, 'OpenAI', lambda **kwargs: object())
    monkeypatch.setattr(model_transport, 'chat_completion', lambda *args, **kwargs:
        SimpleNamespace(choices=[SimpleNamespace(message=SimpleNamespace(
            content='{"verdict":"pass","issues":[]}'), finish_reason='length')]))
    result = qa_agent.call_qa_vlm(
        composite_png=b'png', event_text='Source', road_seed={}, scene_seed={},
        vlm_model='test-model', base_url='https://test.invalid', api_key_env='QA_TEST_KEY')
    assert result['verdict'] == 'fail'
    assert result['vlm_parse_error']
