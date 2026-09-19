from types import SimpleNamespace

import fitz

from tools.extract_source import _extract_pdf_vlm, pdf_narrative_coverage


def make_pdf(tmp_path):
    narrative = ("The autonomous vehicle was traveling westbound. A passenger car approached from behind "
                 "and contacted its rear bumper. The vehicles separated and the passenger car accelerated "
                 "again, making a second contact with the now stationary autonomous vehicle.")
    path = tmp_path / "source.pdf"
    with fitz.open() as document:
        page = document.new_page()
        widget = fitz.Widget()
        widget.field_name = "accident_description"
        widget.field_type = fitz.PDF_WIDGET_TYPE_TEXT
        widget.rect = fitz.Rect(40, 400, 550, 600)
        widget.field_value = narrative
        page.add_widget(widget)
        document.save(path)
    return path, narrative


def test_missing_narrative_fails_even_when_form_header_was_read(tmp_path):
    path, narrative = make_pdf(tmp_path)
    assert pdf_narrative_coverage(path, narrative)["passed"]
    coverage = pdf_narrative_coverage(path, "SECTION 1 Manufacturer, SECTION 2 Vehicle was moving")
    assert coverage["applicable"]
    assert not coverage["passed"]
    assert coverage["fields"][0]["page"] == 1


def test_nonempty_truncated_vlm_output_is_retried(tmp_path, monkeypatch):
    path, narrative = make_pdf(tmp_path)
    monkeypatch.setenv("TEST_VLM_KEY", "test-only")
    monkeypatch.setattr("openai.OpenAI", lambda **kwargs: object())
    monkeypatch.setattr("tools.extract_source._render_pdf_pages_b64", lambda *a, **k: ["test-image"])
    calls = []

    def completion(client, **kwargs):
        calls.append(kwargs["max_tokens"])
        return SimpleNamespace(choices=[SimpleNamespace(
            message=SimpleNamespace(content=narrative), finish_reason="length" if len(calls) == 1 else "stop")])

    monkeypatch.setattr("tools.model_transport.chat_completion", completion)
    assert _extract_pdf_vlm(path, model="test", base_url="unused", api_key_env="TEST_VLM_KEY") == narrative
    assert calls == [8000, 16000]
