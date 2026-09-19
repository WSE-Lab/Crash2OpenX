from __future__ import annotations

import unittest
from pathlib import Path

from tools.api_infer_scene_seed_v2 import normalize


def _raw(snippet: str) -> dict:
    return {
        "status": "supported",
        "scene": {
            "sut": {"id": "ego", "kind": "vehicle", "maneuver": "straight"},
            "npcs": [{
                "id": "v1", "kind": "vehicle", "position": "ahead_same_lane",
                "side": "none", "behavior": {"block": "stopped_ahead"},
            }],
            "collision": {"a": "ego", "b": "v1"},
            "control": "unknown",
        },
        "evidence": {"source_snippets": [snippet]},
    }


class TemporalNormalizationTests(unittest.TestCase):
    def test_abrupt_stop_sequence_becomes_front_brake(self):
        raw = _raw(
            "Traffic ahead slowed abruptly; the lead car came to a complete stop, "
            "and the following car caused a rear-end collision."
        )
        result = normalize(raw, Path("case.txt"), "test")
        behavior = result["scene"]["npcs"][0]["behavior"]
        self.assertEqual(behavior["block"], "front_brake")
        self.assertEqual(
            behavior["params"],
            {"trig_simtime": 2.0, "brake_t": 0.2, "end_speed": 0},
        )
        self.assertEqual(
            result["pipeline"]["normalization_rules"],
            ["abrupt_lead_stop_to_front_brake"],
        )

    def test_abrupt_stop_case165_phrasing_becomes_front_brake(self):
        # Case 165 (demo): "came to an abrupt stop" rather than "slowed abruptly".
        raw = _raw(
            "The vehicle in front of him came to an abrupt stop due to slowed "
            "traffic. The driver brought the vehicle to a stop in his lane; the "
            "vehicle behind did not stop in time and rear ended it."
        )
        result = normalize(raw, Path("case.txt"), "test")
        self.assertEqual(
            result["scene"]["npcs"][0]["behavior"]["block"], "front_brake"
        )

    def test_already_stationary_lead_stays_stopped_ahead(self):
        raw = _raw("A vehicle was parked and already stopped before the following car approached.")
        result = normalize(raw, Path("case.txt"), "test")
        self.assertEqual(
            result["scene"]["npcs"][0]["behavior"]["block"],
            "stopped_ahead",
        )
        self.assertEqual(result["pipeline"]["normalization_rules"], [])

    def test_original_facts_survive_sparse_model_evidence(self):
        raw = _raw("Vehicle 1 was stopped; collision type was rear end.")
        source = {
            "event_description": (
                "Traffic ahead slowed abruptly, the lead vehicle came to a complete stop, "
                "and the following vehicle struck its rear."
            )
        }
        result = normalize(raw, Path("case.txt"), "test", source_context=source)
        self.assertEqual(
            result["scene"]["npcs"][0]["behavior"]["block"],
            "front_brake",
        )

    def test_negated_model_explanation_cannot_invent_abrupt_braking(self):
        raw = _raw("The AV was stopped in traffic for a red light when another car rear ended it.")
        raw['evidence']['reason'] = '在 ego 接近前就已停止，非先行驶后急刹。'
        result = normalize(raw, Path('case.txt'), 'test')
        self.assertEqual(result['scene']['npcs'][0]['behavior']['block'], 'stopped_ahead')
        self.assertEqual(result['pipeline']['normalization_rules'], [])

    def test_original_facts_override_hallucinated_model_abrupt_stop_claim(self):
        raw = _raw("A vehicle braked abruptly to a stop and was rear ended.")
        source = {'event_description': 'The AV was already stopped at the red light before the rear car approached.'}
        result = normalize(raw, Path('case.txt'), 'test', source_context=source)
        self.assertEqual(result['scene']['npcs'][0]['behavior']['block'], 'stopped_ahead')


if __name__ == "__main__":
    unittest.main()
