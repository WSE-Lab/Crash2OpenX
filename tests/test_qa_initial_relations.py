from tools.qa_agent import initial_relations, render_scene_schematic


def seed(npcs):
    return {'scene': {'sut': {'id': 'ego', 'kind': 'vehicle', 'maneuver': 'straight'},
                      'npcs': npcs, 'collision': {'a': 'ego', 'b': npcs[0]['id']}}}


def cyclist(params=None, block='cut_in'):
    return {'id': 'bike', 'kind': 'cyclist', 'position': 'roadside', 'side': 'right',
            'behavior': {'block': block, 'params': params or {}}}


def test_roadside_default_is_ahead_and_explicit_negative_gap_is_behind():
    front = initial_relations(seed([cyclist()]))[0]
    assert front['longitudinal_m'] == 25 and front['dy'] < 0
    assert front['longitudinal_is_default']
    rear = initial_relations(seed([cyclist({'gap': -12})]))[0]
    assert rear['longitudinal_m'] == -12 and rear['dy'] > 0
    assert not rear['longitudinal_is_default']


def test_roadside_cruise_is_alongside_not_default_behind():
    relation = initial_relations(seed([cyclist(block='cruise')]))[0]
    assert relation['longitudinal_m'] == 0 and relation['dy'] == 0


def test_reference_chain_is_preserved_and_renderable():
    follower = cyclist({'gap': -9})
    follower['relative_to'] = 'lead'
    lead = {'id': 'lead', 'kind': 'vehicle', 'position': 'ahead_same_lane', 'side': 'none',
            'behavior': {'block': 'cruise', 'params': {'gap': 20}}}
    value = seed([follower, lead])
    relations = initial_relations(value)
    assert [r['id'] for r in relations] == ['lead', 'bike']
    assert relations[1]['reference'] == 'lead'
    assert relations[1]['compiler_ds_m'] == -9
    assert render_scene_schematic(value).startswith(b'\x89PNG')


def test_negative_roadside_gap_survives_scene_normalization(tmp_path):
    from tools.api_infer_scene_seed_v2 import normalize
    value = seed([cyclist({'gap': -12})])
    value['status'] = 'supported'
    value['scene'].update(control='none', environment={})
    normalized = normalize(value, tmp_path/'source.pdf', 'test-model')
    assert initial_relations(normalized)[0]['longitudinal_m'] == -12
