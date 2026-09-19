"""Prepare the narrow ScenarioRunner parser hook for sinusoidal offsets."""


def patched_parser(source):
    old = '''atomic = ChangeActorLaneOffset(
                            actor, absolute_offset, continuous=continuous, name=maneuver_name)'''
    new = '''dynamics = lat_maneuver.find('LaneOffsetActionDynamics')
                        if dynamics is not None and dynamics.get('dynamicsShape') == 'sinusoidal':
                            from lane_offset_motion import SinusoidalLaneOffset
                            atomic = SinusoidalLaneOffset(
                                actor, absolute_offset, ParameterRef(dynamics.get('maxLateralAcc')),
                                continuous=continuous,
                                use_carla_coordinates=OpenScenarioParser.use_carla_coordinate_system,
                                name=maneuver_name)
                        else:
                            atomic = ChangeActorLaneOffset(
                                actor, absolute_offset, continuous=continuous, name=maneuver_name)'''
    if source.count(old) != 1:
        raise ValueError('Expected one unmodified absolute lane-offset parser branch')
    changed = source.replace(old, new)
    compile(changed, 'openscenario_parser.py', 'exec')
    return changed


def patched_zero_relative_station(source):
    old = '''                    else:
                        waypoint = waypoint.next(ds)[-1]'''
    new = '''                    elif ds > 0:
                        waypoint = waypoint.next(ds)[-1]'''
    if source.count(old) != 1:
        raise ValueError('Expected one relative station branch')
    changed = source.replace(old, new)
    compile(changed, 'openscenario_parser.py', 'exec')
    return changed
