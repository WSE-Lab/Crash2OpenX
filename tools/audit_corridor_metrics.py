"""Review every selected fresh run without rewriting raw CARLA metrics."""
import argparse
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path

from tools.xodr_corridor_review import review_trace


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('snapshot', type=Path)
    ap.add_argument('output', type=Path)
    args = ap.parse_args()
    manifest = json.loads((args.snapshot/'manifest.json').read_text())
    args.output.mkdir(parents=True, exist_ok=False)
    results = []
    files = {f['archive_path']: f for f in manifest['files']}
    for case in manifest['cases']:
        cid = case['case_id']
        paths = {}
        for name in ('runtime_map.xodr', 'sim_trace_raw.jsonl', 'summary.json'):
            entry = files[cid+'/'+name]; path = Path(entry['source_path'])
            if hashlib.sha256(path.read_bytes()).hexdigest() != entry['sha256']:
                raise ValueError('Snapshot source changed: '+str(path))
            paths[name] = path
        rows = [json.loads(line) for line in paths['sim_trace_raw.jsonl'].read_text().splitlines()]
        summary = json.loads(paths['summary.json'].read_text())
        try:
            reviewed = review_trace(paths['runtime_map.xodr'], rows)
        except (KeyError, ValueError, TypeError) as exc:
            reviewed = {'error': str(exc), 'samples': len(rows), 'source_fidelity_accepted': False}
        reviewed.update(case_id=cid, attempt=case['selected_attempt'],
                        trace_sha256=files[cid+'/sim_trace_raw.jsonl']['sha256'],
                        raw_offroad_seconds=summary.get('off_road_time'))
        (args.output/(cid+'.json')).write_text(json.dumps(reviewed, indent=2)+'\n')
        results.append({k: reviewed.get(k) for k in ('case_id', 'attempt', 'samples', 'inside_samples',
            'outside_samples', 'unresolved_samples', 'raw_offroad_seconds', 'error')})
        print(json.dumps(results[-1]), flush=True)
    (args.output/'summary.json').write_text(json.dumps({
        'created_at': datetime.now(timezone.utc).isoformat(), 'snapshot': str(args.snapshot.resolve()),
        'cases': results, 'raw_metrics_overwritten': False, 'source_fidelity_accepted': False,
        'code_sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in
                       (Path(__file__), Path(__file__).with_name('xodr_corridor_review.py'))}}, indent=2)+'\n')


if __name__ == '__main__':
    main()
