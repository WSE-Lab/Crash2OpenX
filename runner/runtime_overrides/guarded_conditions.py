"""Evaluate event predicates only after storyboard prerequisites complete.

ScenarioRunner otherwise latches every predicate independently from t=0.
A future 'target stopped' trigger can then accidentally remember spawn rest.
"""
import py_trees


def guarded_condition_group(condition_group, convert):
    conditions = list(condition_group.findall('Condition'))
    prerequisites = [c for c in conditions if c.find('.//StoryboardElementStateCondition') is not None]
    predicates = [c for c in conditions if c not in prerequisites]
    if not prerequisites or not predicates:
        return None
    sequence = py_trees.composites.Sequence(name='Storyboard prerequisite then current predicates')
    for label, children in [('Storyboard prerequisites', prerequisites), ('Current predicates', predicates)]:
        group = py_trees.composites.Parallel(
            name=label, policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
        for condition in children:
            group.add_child(convert(condition))
        sequence.add_child(group)
    return sequence
