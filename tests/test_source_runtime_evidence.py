from tools.source_runtime_evidence import select_contact_episodes


def test_pcla_navigation_preserves_source_turn_instead_of_replacing_it_with_straight_route(tmp_path):
    import json
    from tools.validate_source_reconstruction import prepare
    output = tmp_path / "turn"
    prepare("144_Waymo_October_17_2023", output, "pcla")
    document = json.loads((output / "trace_reference.json").read_text())
    route = document["actors"]["hero"]["trace"]
    assert len(route) > 10
    assert abs(((route[-1]["h"] - route[0]["h"] + 180) % 360 - 180) + 90) < 1e-6


def contact(t, a="hero", b="other"):
    return {"event_type": "collision", "simulation_time": t,
            "payload": {"source": "carla_collision_sensor", "actors": [a, b]}}


def test_sustained_contact_cannot_satisfy_two_impact_source():
    events = [contact(t / 20) for t in range(20, 80)]
    first, second = select_contact_episodes(events, [["hero", "other"], ["hero", "other"]])
    assert first["simulation_time"] == 1
    assert second is None


def test_contact_gap_selects_separate_episode_and_preserves_pair_order():
    events = [contact(1, "pickup", "middle"), contact(1.05, "middle", "pickup"), contact(2, "middle", "hero")]
    chosen = select_contact_episodes(events, [["pickup", "middle"], ["middle", "hero"]])
    assert [e["simulation_time"] for e in chosen] == [1, 2]
    chosen = select_contact_episodes([contact(1), contact(1.1), contact(3)], [["hero", "other"], ["hero", "other"]])
    assert [e["simulation_time"] for e in chosen] == [1, 3]
