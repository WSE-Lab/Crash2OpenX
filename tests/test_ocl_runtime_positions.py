from tools.ocl_constraints import evaluate_constraints, paper_example


def test_runtime_adjacent_position_cannot_bypass_lane_count():
    road, scene = paper_example()
    road["lanes"] = {"forward": 1, "backward": 1}
    scene["npcs"][0]["position"] = "adjacent"
    scene["npcs"][0]["side"] = "right"
    assert not evaluate_constraints(road, scene)["P4"]
    road["lanes"]["forward"] = 2
    assert evaluate_constraints(road, scene)["P4"]


def test_runtime_roadside_pedestrian_uses_separate_side_field():
    road, scene = paper_example()
    scene["npcs"][0].update({"kind": "pedestrian", "position": "roadside", "side": "left"})
    assert evaluate_constraints(road, scene)["I3"]
