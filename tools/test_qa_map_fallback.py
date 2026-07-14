#!/usr/bin/env python3
"""Regression tests for browser-free QA map rendering and arc sampling."""

import io
import tempfile
import unittest
from pathlib import Path

from PIL import Image

from tools.qa_agent import _render_map_png_xodr
from tools.replay_geom import parse_xodr_lines


XODR = """<?xml version="1.0"?>
<OpenDRIVE>
  <road id="1" length="10" junction="-1">
    <planView><geometry s="0" x="0" y="0" hdg="0" length="10"><line/></geometry></planView>
  </road>
  <road id="2" length="20" junction="-1">
    <planView><geometry s="0" x="10" y="0" hdg="0" length="20"><arc curvature="0.05"/></geometry></planView>
  </road>
</OpenDRIVE>
"""


class QaMapFallbackTest(unittest.TestCase):
    def test_arc_is_sampled_and_rendered(self):
        with tempfile.TemporaryDirectory() as tmp:
            html = Path(tmp) / "case.html"
            html.write_text("<html></html>")
            html.with_suffix(".xodr").write_text(XODR)

            roads = parse_xodr_lines(html.with_suffix(".xodr"))
            self.assertEqual(len(roads), 2)
            self.assertGreater(len(roads[1]["points"]), 3)
            self.assertGreater(roads[1]["points"][-1][1], 0)

            png = _render_map_png_xodr(
                html,
                {"road": {"topology": "curve", "type": "motorway",
                           "lanes": {"forward": 1, "backward": 0},
                           "center_line": "broken"}},
                640,
                400,
            )
            image = Image.open(io.BytesIO(png))
            self.assertEqual(image.size, (640, 400))
            self.assertGreater(len(image.getcolors(maxcolors=1_000_000)), 20)


if __name__ == "__main__":
    unittest.main()
