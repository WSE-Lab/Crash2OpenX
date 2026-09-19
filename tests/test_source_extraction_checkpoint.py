import hashlib
import json

import pytest

from tools.extract_source import load_extraction_checkpoint


@pytest.mark.parametrize("tamper", [None, "source", "text", "model"])
def test_checkpoint_rejects_changed_evidence(tmp_path, tamper):
    source = tmp_path / "source.txt"
    source.write_bytes(b"original PDF bytes")
    text = tmp_path / "source_text.txt"
    text.write_text("Facts from the original PDF")
    record = {"source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
              "text_sha256": hashlib.sha256(text.read_bytes()).hexdigest(),
              "vlm_model": "vlm", "base_url": "https://test.invalid", "extracted_at": "2026-09-17"}
    (tmp_path / "source_extraction.json").write_text(json.dumps(record))
    if tamper == "source":
        source.write_bytes(b"another PDF")
    elif tamper == "text":
        text.write_text("altered facts")
    kwargs = {"model": "different" if tamper == "model" else "vlm", "base_url": "https://test.invalid"}
    if tamper:
        with pytest.raises(ValueError, match="checkpoint mismatch"):
            load_extraction_checkpoint(source, tmp_path, **kwargs)
    else:
        actual, evidence = load_extraction_checkpoint(source, tmp_path, **kwargs)
        assert actual == text.read_text()
        assert evidence == record
