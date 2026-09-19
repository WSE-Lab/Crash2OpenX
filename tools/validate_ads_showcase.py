#!/usr/bin/env python3
"""Independently verify a portable showcase against its original evidence."""
import argparse
import hashlib
from html.parser import HTMLParser
import json
from pathlib import Path
import subprocess
from urllib.parse import unquote, urlsplit
from xml.etree import ElementTree as ET
import zipfile

import xmlschema

ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    with path.open('rb') as handle:
        return hashlib.file_digest(handle, 'sha256').hexdigest()


class Links(HTMLParser):
    def __init__(self):
        super().__init__()
        self.paths = []

    def handle_starttag(self, tag, attrs):
        for name, value in attrs:
            if name in {'href', 'src', 'poster'} and value:
                url = urlsplit(value)
                if not url.scheme and url.path:
                    self.paths.append(unquote(url.path))


def verify(output, archive=True):
    failures = []
    def check(condition, message):
        if not condition:
            failures.append(message)
    selection = json.loads((output/'selection.json').read_text())
    records = selection['cases']
    check(len({r['case_id'] for r in records}) >= 10, 'Fewer than ten distinct cases')
    manifest = json.loads((output/'MANIFEST.json').read_text())['files']
    actual = {str(p.relative_to(output)) for p in output.rglob('*') if p.is_file()}
    check(actual == set(manifest) | {'MANIFEST.json'}, 'Manifest coverage mismatch')
    for name, expected in manifest.items():
        check(digest(output/name) == expected, 'Hash mismatch: '+name)
    parser = Links()
    parser.feed((output/'START_HERE.html').read_text())
    for path in parser.paths:
        check((output/path).is_file(), 'Broken gallery link: '+path)
    osc_schema = xmlschema.XMLSchema(ROOT/'xsd/OpenSCENARIO.xsd')
    odr_schema = xmlschema.XMLSchema(ROOT/'xsd/OpenDRIVE_1.5M.xsd')
    decoded = []
    for record in records:
        folder = output/'cases'/record['case_id']
        for schema, name in [(osc_schema, 'scenario.xosc'), (odr_schema, 'map.xodr')]:
            check(schema.is_valid(folder/name), 'XML schema failure: '+str(folder/name))
        tree = ET.parse(folder/'scenario.xosc')
        check(tree.find('.//RoadNetwork/LogicFile').get('filepath') == 'map.xodr', 'Map link is not portable')
        quality = json.loads((folder/'quality_review.json').read_text())
        check(quality['candidate_for_visual_review'], 'Mechanical quality gate failed')
        check(quality['native_map']['verified'], 'Native map unverified')
        remote = json.loads((folder/'evidence/remote_run.json').read_text())
        check(remote['xodr_sha256'] == digest(folder/'map.xodr'), 'Map differs from actual uploaded input')
        check(remote['xosc_sha256'] == digest(folder/'scenario.xosc'), 'Scenario differs from actual uploaded input')
        check(remote['pcla_agent'] == 'if_if' and remote['sut_actor'] == 'hero', 'Wrong ADS identity')
        source = json.loads((folder/'manifest.json').read_text())
        check(source['source_scene_sha256'] == digest(folder/'source/scene_seed.json'), 'Source scene hash mismatch')
        runtime = json.loads((folder/'evidence/runtime_manifest.json').read_text())
        for name, expected in runtime['files'].items():
            check(digest(folder/'evidence/runtime_code'/name) == expected, 'Runtime code mismatch: '+name)
        clip = json.loads((folder/'clip_provenance.json').read_text())
        for name, key in [('interaction.mp4', 'clip_sha256'), ('evidence/carla_rgb.mp4', 'raw_video_sha256'),
                          ('evidence/ads_review/ads_review.mp4', 'source_video_sha256')]:
            check(digest(folder/name) == clip[key], 'Video provenance mismatch: '+name)
            result = subprocess.run(['ffmpeg', '-v', 'error', '-i', str(folder/name), '-f', 'null', '-'],
                                    capture_output=True, text=True)
            check(result.returncode == 0 and not result.stderr.strip(), 'Video decode failure: '+str(folder/name))
            decoded.append(str((folder/name).relative_to(output)))
    result = subprocess.run(['ffmpeg', '-v', 'error', '-i', str(output/'showcase.mp4'), '-f', 'null', '-'],
                            capture_output=True, text=True)
    check(result.returncode == 0 and not result.stderr.strip(), 'Montage decode failure')
    if archive:
        path = output.with_suffix('.zip')
        check(digest(path) == path.with_suffix('.zip.sha256').read_text().split()[0], 'ZIP hash mismatch')
        with zipfile.ZipFile(path) as bundle:
            check(bundle.testzip() is None, 'ZIP CRC failure')
            check(set(bundle.namelist()) == {output.name+'/'+p for p in actual}, 'ZIP file list mismatch')
            for info in bundle.infolist():
                local = output.parent/info.filename
                with bundle.open(info) as handle:
                    check(hashlib.file_digest(handle, 'sha256').hexdigest() == digest(local), 'ZIP differs: '+info.filename)
    report = {'passed': not failures, 'distinct_cases': len({r['case_id'] for r in records}),
              'verified_files': len(manifest), 'gallery_local_links': len(parser.paths),
              'decoded_videos': len(decoded)+1, 'xml_schema_pairs': len(records),
              'runtime_hashes_verified': True, 'archive_verified': archive, 'failures': failures}
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    result = verify(args.directory.resolve())
    args.report.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(result, ensure_ascii=False))
    raise SystemExit(0 if result['passed'] else 1)
