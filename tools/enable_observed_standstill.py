"""Narrow parser hook for a complete observed StandStillCondition interval."""


def patched_parser(source):
    old = 'atomic = StandStill(trigger_actor, condition_name, duration)'
    new = ('from standstill_condition import ObservedStandStill\n'
           '                    atomic = ObservedStandStill(trigger_actor, condition_name, duration)')
    if source.count(old) != 1:
        raise ValueError('Expected one native StandStillCondition parser branch')
    changed = source.replace(old, new)
    compile(changed, 'openscenario_parser.py', 'exec')
    return changed
