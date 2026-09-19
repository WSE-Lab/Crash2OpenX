"""Explicit, live ADS-relative action gates (distances are body clearances).

These are experimental scenario parameters, not facts inferred from a crash
report. Existing blocks keep their original triggers when this field is absent.
"""
import math

import scenariogeneration.xosc as xosc

PARAMETER = 'C2XADSTriggers'
PREFIX = 'c2x_ads_'
EVENT_SUFFIX = {'front_brake': 'brake', 'cut_in': 'cut',
                'partial_lane_intrusion': 'partial_intrusion', 'cross': 'move',
                'junction_cross': 'depart', 'junction_turn': 'depart'}


def normalize_ads_trigger(npc):
    behavior = npc.get('behavior') or {}
    if 'ads_trigger' not in behavior:
        return None
    block = behavior.get('block')
    if block not in EVENT_SUFFIX:
        raise ValueError(f'ads_trigger is not supported for {block}')
    raw = behavior['ads_trigger']
    allowed = {'distance_m', 'min_clearance_m', 'min_ego_speed_mps',
               'min_npc_speed_mps', 'min_closing_speed_mps'}
    if not isinstance(raw, dict) or set(raw) - allowed or 'distance_m' not in raw:
        raise ValueError('ads_trigger requires distance_m and only documented fields')
    result = {'min_clearance_m': 0.5, 'min_ego_speed_mps': 1.0,
              'min_npc_speed_mps': 2.0 if block in {'cut_in', 'partial_lane_intrusion'} else 0.0,
              **raw}
    for key, value in result.items():
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
            raise ValueError(f'ads_trigger.{key} must be finite and non-negative')
    if not 0.5 <= result['min_clearance_m'] < result['distance_m']:
        raise ValueError('ads_trigger requires 0.5 <= min_clearance_m < distance_m')
    if block in {'cut_in', 'partial_lane_intrusion'} and result['min_npc_speed_mps'] < 1.0:
        raise ValueError('lateral actions require min_npc_speed_mps >= 1')
    if block in {'cross', 'junction_cross', 'junction_turn'} and result['min_npc_speed_mps']:
        raise ValueError('a waiting crossing actor cannot require a moving NPC')
    if 'min_closing_speed_mps' in result:
        # RelativeSpeedCondition compares speed magnitudes, not velocity vectors.
        # It is valid as a closing-speed gate only for same-direction approaches.
        if npc.get('position') not in {'ahead_same_lane', 'adjacent'} or block not in {'front_brake', 'cut_in', 'partial_lane_intrusion'}:
            raise ValueError('min_closing_speed_mps requires a same-direction lead/adjacent actor')
        if npc.get('relative_to', 'ego') != 'ego':
            raise ValueError('closing-speed gate requires position relative to ego')
    legacy = set((behavior.get('params') or {})) & {'trig_dist', 'trig_ttc', 'trig_simtime', 'trig_simtime_min'}
    if legacy:
        raise ValueError('ads_trigger conflicts with legacy trigger parameters: ' + ', '.join(sorted(legacy)))
    return result


def build_ads_trigger(npc):
    config = normalize_ads_trigger(npc)
    if config is None:
        raise ValueError('missing ads_trigger')
    nid = npc['id']
    group = xosc.ConditionGroup()

    def add(suffix, condition, actor='hero'):
        # No independently latched rising edges: the whole window is live AND.
        group.add_condition(xosc.EntityTrigger(PREFIX+nid+'_'+suffix, 0,
                            xosc.ConditionEdge.none, condition, actor))

    for suffix, key, rule in [('near', 'distance_m', xosc.Rule.lessThan),
                               ('clear', 'min_clearance_m', xosc.Rule.greaterThan)]:
        add(suffix, xosc.RelativeDistanceCondition(config[key], rule,
            xosc.RelativeDistanceType.cartesianDistance, nid, freespace=True))
    add('ego_moving', xosc.SpeedCondition(config['min_ego_speed_mps'], xosc.Rule.greaterThan))
    if config['min_npc_speed_mps']:
        add('npc_moving', xosc.SpeedCondition(config['min_npc_speed_mps'], xosc.Rule.greaterThan), nid)
    if 'min_closing_speed_mps' in config:
        add('closing', xosc.RelativeSpeedCondition(config['min_closing_speed_mps'], xosc.Rule.greaterThan, nid))
    trigger = xosc.Trigger()
    trigger.add_conditiongroup(group)
    return trigger


def trigger_manifest(scene):
    plans = {}
    for npc in scene.get('npcs', []):
        config = normalize_ads_trigger(npc)
        if config is not None:
            plans[npc['id']] = {'reference_actor': 'hero', 'event': npc['id']+'_'+EVENT_SUFFIX[npc['behavior']['block']],
                               'distance_semantics': 'cartesian_freespace', **config}
    return plans
