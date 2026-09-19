from tools.road_recompile_equivalence import equivalent_roads


def test_metadata_and_lane_link_order_are_the_only_ignored_structure(tmp_path):
    original = tmp_path / 'old.xodr'
    rebuilt = tmp_path / 'new.xodr'
    original.write_text('''<OpenDRIVE><header name="old" date="today" revMajor="1"/>
      <junction id="1"><connection id="2" incomingRoad="3" connectingRoad="4">
      <laneLink from="-1" to="-1"/><laneLink from="-2" to="-2"/>
      </connection></junction></OpenDRIVE>''')
    rebuilt.write_text('''<OpenDRIVE><header name="new" date="later" revMajor="1"/>
      <junction id="1"><connection id="2" incomingRoad="3" connectingRoad="4">
      <laneLink from="-2" to="-2"/><laneLink from="-1" to="-1"/>
      </connection></junction></OpenDRIVE>''')
    assert equivalent_roads(original, rebuilt)['equivalent']
    rebuilt.write_text(rebuilt.read_text().replace('connectingRoad="4"', 'connectingRoad="5"'))
    assert not equivalent_roads(original, rebuilt)['equivalent']


def test_constant_spiral_may_become_arc_but_curvature_cannot_change(tmp_path):
    original, rebuilt = tmp_path / 'old.xodr', tmp_path / 'new.xodr'
    prefix = '<OpenDRIVE><road><planView><geometry s="0" x="0" y="0" hdg="0" length="10">'
    suffix = '</geometry></planView></road></OpenDRIVE>'
    original.write_text(prefix+'<spiral curvStart="0.01" curvEnd="0.01"/>'+suffix)
    rebuilt.write_text(prefix+'<arc curvature="0.01"/>'+suffix)
    assert equivalent_roads(original, rebuilt)['equivalent']
    rebuilt.write_text(prefix+'<arc curvature="0.0101"/>'+suffix)
    assert not equivalent_roads(original, rebuilt)['equivalent']
