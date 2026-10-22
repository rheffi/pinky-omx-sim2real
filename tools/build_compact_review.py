"""Build standalone v4 review; never updates v3, the control repo or a chat."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from pinky_motion import compact_routes, route_paths
from pinky_motion.compact_reference import run_pipeline
from pinky_motion.fleet_reference import TimedPath
from pinky_motion.route_validation import check_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.resolve().is_relative_to((ROOT/'deliverables').resolve()):
        raise ValueError('Frozen deliverables must not be used as review output')
    args.output.mkdir(parents=True, exist_ok=True)
    evidence = args.output/'evidence'; evidence.mkdir(exist_ok=True)
    config = json.loads((ROOT/'config/layout.json').read_text(encoding='utf-8'))
    paths = compact_routes.route_set(config)
    checks = {key: check_path(config, route_paths.join(s)) for key, s in paths.items()}
    if not all(c['valid'] for c in checks.values()): raise RuntimeError('Invalid geometry')
    runs = {'normal': run_pipeline(config), 'stale': run_pipeline(config, fault=True),
            'skip': run_pipeline(config, orders=(('A',), ('B',))),
            'baseline': run_pipeline(config, version=3)}
    if not all(all(r['checks'].values()) for r in runs.values()): raise RuntimeError('Invalid scenario')
    metrics = {}
    for a, b in [('A', 'B'), ('B', 'handoff'), ('A', 'handoff')]:
        metrics[a+'_to_'+b] = {f'v{v}': TimedPath(api.transfer(config, a, b)).distance[-1]
                              for v, api in [(3, route_paths), (4, compact_routes)]}
    summary = {'candidate': True, 'control_session_updated': False, 'route_version': 4,
               'physical_simulation': False, 'margin_m': .02, 'geometry': checks,
               'segment_distance_m': metrics, 'scenarios': {}, 'passed': True}
    ui_runs = {}
    for name, result in runs.items():
        frames = result.pop('frames')
        summary['scenarios'][name] = result
        (evidence/(name+'-frames.json')).write_text(json.dumps(frames, separators=(',', ':')), encoding='utf-8')
        reduced = frames[::4]
        if reduced[-1] is not frames[-1]: reduced.append(frames[-1])
        ui_runs[name] = dict(result, frames=reduced)
    old = {'queue_advance': [route_paths.home_paths()['advance']]}
    for station in route_paths.STATIONS: old['home_to_'+station] = route_paths.home_to_station(config, station)
    for a,b in [('A','B'), ('A','handoff'), ('B','handoff')]: old[a+'_to_'+b] = route_paths.transfer(config,a,b)
    for slot in ('front','back'): old['handoff_to_home_'+slot] = route_paths.station_to_home(config,'handoff',slot)
    ui_paths = {str(v): {key: [[round(n,6) for n in p] for p in route_paths.join(s)]
                        for key,s in collection.items()}
                for v,collection in [(4,paths),(3,old)]}
    data = dict(config=config, summary=summary, runs=ui_runs, paths=ui_paths)
    html = (ROOT/'tools/compact_review_template.html').read_text(encoding='utf-8')
    html = html.replace('__REVIEW_DATA__', json.dumps(data, ensure_ascii=False, separators=(',', ':')))
    (args.output/'replay.html').write_text(html, encoding='utf-8')
    (evidence/'result.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
    (evidence/'layout.json').write_text(json.dumps(config, ensure_ascii=False, indent=2), encoding='utf-8')
    (evidence/'paths.json').write_text(json.dumps(ui_paths, separators=(',', ':')), encoding='utf-8')
    source_names = ['pinky_motion/compact_routes.py','pinky_motion/compact_reference.py',
                    'pinky_motion/route_paths.py','pinky_motion/route_validation.py',
                    'pinky_motion/geometry.py','pinky_motion/fleet_reference.py',
                    'tools/build_compact_review.py','tools/compact_review_template.html',
                    'tests/test_compact_review.py','config/layout.json']
    hashes = {p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in source_names}
    (evidence/'source-hashes.json').write_text(json.dumps(hashes, indent=2), encoding='utf-8')
    print(json.dumps({'passed': True, 'review':str(args.output/'replay.html'),
                      'v4_seconds':runs['normal']['elapsed_sim_seconds'],
                      'v3_same_conditions_seconds':runs['baseline']['elapsed_sim_seconds'],
                      'geometry_routes':len(checks)}, ensure_ascii=False))


if __name__ == '__main__': main()
