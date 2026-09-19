"""Optional curb parking bands, separate from the traffic-lane count."""

PARKING_WIDTH_M = 2.5


def normalize_parking(road):
    value = road.get('parking', {})
    if not isinstance(value, dict) or set(value) - {'left', 'right'}:
        raise ValueError('road.parking permits only left/right booleans')
    if any(type(v) is not bool for v in value.values()):
        raise ValueError('road.parking sides must be booleans')
    if any(value.values()):
        if road.get('topology') != 'straight':
            raise ValueError('parking bands currently require a straight road')
        if value.get('left') and road.get('lanes', {}).get('backward') != 0:
            raise ValueError('left curb parking beside forward traffic requires a one-way road')
    return dict(value)
