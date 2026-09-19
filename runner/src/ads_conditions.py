"""Keep ADS-relative predicate windows live until the containing event starts."""
import py_trees
from ads_geometry import hull, body_clearance


class BodyDistanceCondition(py_trees.behaviour.Behaviour):
    """Correct cartesian freespace distance for angled and adjacent vehicles.

    Stock ScenarioRunner subtracts two half-lengths from center distance,
    incorrectly reporting adjacent non-touching cars as overlapping.
    """
    def __init__(self, native):
        super().__init__(native.name)
        self.native = native

    def update(self):
        native = self.native
        actors = (native._actor, native._reference_actor)
        if not all(actor.is_alive for actor in actors):
            return py_trees.common.Status.RUNNING
        polygons = [hull([(v.x, v.y) for v in actor.bounding_box.get_world_vertices(actor.get_transform())])
                    for actor in actors]
        distance = body_clearance(*polygons)
        return (py_trees.common.Status.SUCCESS if native._comparison_operator(distance, native._distance)
                else py_trees.common.Status.RUNNING)


def live_ads_condition_group(condition_group, convert):
    conditions = list(condition_group.findall('Condition'))
    tagged = [c.get('name', '').startswith('c2x_ads_') for c in conditions]
    if not any(tagged):
        return None
    if not all(tagged):
        raise ValueError('ADS trigger conditions must occupy their own live AND group')
    group = py_trees.composites.Parallel(
        name='Live ADS relative window', policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
    for condition in conditions:
        native = convert(condition)
        distance = condition.find('.//RelativeDistanceCondition')
        if distance is not None and distance.get('freespace') == 'true' and distance.get('relativeDistanceType') == 'cartesianDistance':
            native = BodyDistanceCondition(native)
        group.add_child(native)
    return group
