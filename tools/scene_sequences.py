"""Validate a small, explicit NPC action sequence vocabulary."""
import math


ACTIONS = {
    'drive': {'speed', 'duration'},
    'match_speed': {'duration'},
    'brake': {'end_speed', 'duration'},
    'lane_change': {'direction', 'lanes', 'distance'},
}
CONDITIONS = {'start', 'contact', 'after_previous', 'separated_and_target_stopped'}


def normalize_steps(steps):
    if not isinstance(steps, list) or len(steps) < 2:
        raise ValueError('sequence requires at least two explicit steps')
    result = []
    for index, step in enumerate(steps):
        if not isinstance(step, dict) or set(step)-{'action', 'target', 'params', 'when'}:
            raise ValueError('unsupported sequence step fields')
        action = step.get('action')
        if action not in ACTIONS:
            raise ValueError('unsupported sequence action')
        params = step.get('params') or {}
        when = step.get('when') or {}
        condition = when.get('condition')
        if not isinstance(params, dict) or set(params)-ACTIONS[action]:
            raise ValueError('unsupported sequence action parameters')
        if not isinstance(when, dict) or set(when)-{'condition', 'target', 'delay', 'clearance', 'standstill_duration'}:
            raise ValueError('unsupported sequence trigger parameters')
        if condition not in CONDITIONS or (index == 0) != (condition == 'start'):
            raise ValueError(f'step {index+1}: invalid when.condition={condition!r}; '
                             'step 1 must use start; later steps use contact, after_previous, '
                             'or separated_and_target_stopped')
        if action == 'lane_change':
            if params.get('direction') not in {'left', 'right'}:
                raise ValueError('lane_change requires left/right direction')
            if type(params.get('lanes', 1)) is not int or not 1 <= params.get('lanes', 1) <= 5:
                raise ValueError('lane_change lanes must be an integer from 1 to 5')
            distance = params.get('distance', 20)
            if not isinstance(distance, (int, float)) or isinstance(distance, bool) or not math.isfinite(distance) or distance <= 0:
                raise ValueError('lane_change distance must be positive')
            if index == 0:
                raise ValueError('lane_change requires a preceding drive step')
        for key, value in {**params, **{k:v for k,v in when.items() if k not in {'condition','target'}}}.items():
            if key == 'direction':
                continue
            if (not isinstance(value, (int,float)) or isinstance(value, bool) or not math.isfinite(value)
                    or value < 0 or (key in {'duration', 'standstill_duration'} and value == 0)):
                raise ValueError('sequence numeric parameters must be finite and non-negative (durations positive)')
        item = {'action': action, 'params': dict(params), 'when': dict(when)}
        if action == 'match_speed':
            if not isinstance(step.get('target'), str) or not step['target']:
                raise ValueError('match_speed requires a target actor')
            item['target'] = step['target']
        elif 'target' in step:
            raise ValueError('only match_speed uses an action target')
        if condition in {'contact', 'separated_and_target_stopped'} and not isinstance(when.get('target'), str):
            raise ValueError('sequence condition requires a target actor')
        result.append(item)
    return result


def validate_sequence_targets(npcs, actor_ids):
    for npc in npcs:
        behavior = npc.get('behavior') or {}
        if behavior.get('block') != 'sequence':
            continue
        for step in normalize_steps(behavior.get('steps')):
            for target in (step.get('target'), step['when'].get('target')):
                if target is not None and (target not in actor_ids or target == npc['id']):
                    raise ValueError('sequence target must be a different declared actor')
