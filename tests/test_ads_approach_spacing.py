from tools.expand_ads_batch import space_approach


def test_moving_hazard_keeps_same_lane_cyclist_ahead():
    def actor(name, long):
        return {'id': name, 'position': 'adjacent', 'side': 'right',
                'behavior': {'params': {'long': long}}}
    car, cyclist, relative = actor('car', 5), actor('cyclist', 15), actor('relative', 10)
    relative['relative_to'] = 'car'
    scene = {'npcs': [car, cyclist, relative]}
    space_approach(scene, ['car'])
    assert car['behavior']['params']['long'] == 24
    assert cyclist['behavior']['params']['long'] == 34
    assert relative['behavior']['params']['long'] == 10
