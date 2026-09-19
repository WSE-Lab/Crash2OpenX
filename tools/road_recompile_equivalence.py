"""Strict equivalence gate for recompiling a verified current-batch RoadSeed.

Only metadata, formatting, unordered junction lane links and the compiler's
bounded constant-curvature representation are normalized. Connectivity,
lane widths, road marks, geometry and all other attributes remain exact.
"""
import hashlib
import json
from pathlib import Path
from xml.etree import ElementTree as ET

from tools.opendrive_repair import _canonicalize_constant_spirals


def equivalent_roads(original: Path, rebuilt: Path):
    def normalized(path):
        root = ET.parse(path).getroot()
        replacements = _canonicalize_constant_spirals(root)
        header = root.find('header')
        if header is not None:
            for key in ('name', 'date'):
                header.attrib.pop(key, None)

        def structure(element):
            children = [structure(child) for child in element]
            if element.tag == 'connection' and all(c.tag == 'laneLink' for c in element):
                children.sort(key=lambda c: json.dumps(c, sort_keys=True))
            return [element.tag, sorted(element.attrib.items()),
                    (element.text or '').strip(), children]

        encoded = json.dumps(structure(root), ensure_ascii=False, separators=(',', ':')).encode()
        return hashlib.sha256(encoded).hexdigest(), replacements

    old_hash, old_replacements = normalized(original)
    new_hash, new_replacements = normalized(rebuilt)
    return {
        'equivalent': old_hash == new_hash,
        'normalization_version': 1,
        'original_structure_sha256': old_hash,
        'rebuilt_structure_sha256': new_hash,
        'original_constant_spiral_replacements': old_replacements,
        'rebuilt_constant_spiral_replacements': new_replacements,
        'ignored_metadata': ['header.name', 'header.date', 'XML formatting whitespace'],
        'unordered_elements': ['junction/connection/laneLink'],
        'geometry_policy': 'Only compiler-bounded constant spiral conversion; all other values exact',
    }
