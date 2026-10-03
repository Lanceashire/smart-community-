#!/usr/bin/env python3
"""Independently audit missed green transitions in sampled physical patrol logs."""
import argparse
import hashlib
import json
from pathlib import Path
from summarize_performance import summarize

ROOT = Path(__file__).resolve().parents[1]

def inspect_lap(path):
    raw = json.loads(path.read_text())
    records = raw['records']
    gates = []
    for approach, stop in [('top_approach', 'top_stop'), ('lower_approach', 'lower_stop')]:
        start = next(i for i, r in enumerate(records)
                     if r['status'].startswith(('PLAN ' + approach, 'GO ' + approach)))
        end = next(i for i in range(start, len(records))
                   if records[i]['status'].startswith('REACHED ' + stop))
        previous = None
        greens = []
        for r in records[start:end + 1]:
            phase = r['phase']
            if phase == 'unknown':
                continue
            if phase == 'green' and previous in ('red', 'yellow'):
                greens.append(r['t'])
            previous = phase
        # A green already active at the beginning of the approach is not a
        # witnessed new transition. Waiting for a subsequent one is intentional.
        release = records[end]['t']
        waits = [r for r in records[start:end + 1] if r['status'].startswith('STOP ' + stop)]
        delay = release - greens[0] if greens else None
        gates.append({'gate': stop, 'approach_sim_s': records[start]['t'],
                      'new_green_transitions_sim_s': greens, 'release_sim_s': release,
                      'stop_state_wait_s': round(release - waits[0]['t'], 3) if waits else 0.,
                      'release_delay_from_first_new_green_s': round(delay, 3) if delay is not None else None,
                      'extra_green_cycles': max(0, len(greens) - 1),
                      'first_new_green_release': len(greens) == 1 and records[end]['phase'] == 'green',
                      'release_within_four_seconds': delay is not None and 0 <= delay <= 4.})
    metrics = summarize(path)
    return {'source': str(path.relative_to(ROOT)), 'run_id': raw['run_id'], 'lap_id': raw['lap_id'],
            'physical_audit_passed': raw['passed'], 'completion_sim_s': metrics['completion_sim_s'],
            'position_error_max_m': metrics['position_error_max_m'],
            'checks': raw['checks'], 'counts': raw['counts'], 'plates': raw['plates'],
            'whole_body_crossings': raw['whole_body_crossings'], 'gates': gates,
            'passed': raw['passed'] and all(g['first_new_green_release'] and g['release_within_four_seconds'] for g in gates)}

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--folder', default='validation/signal-optimization-2026-10-03')
    args = parser.parse_args()
    folder = ROOT / args.folder
    baseline = inspect_lap(ROOT / 'validation/performance-2026-10-02/candidate/lap-001.json')
    reports = [inspect_lap(p) for p in sorted((folder / 'final').glob('lap-*.json'))
               if not p.name.endswith('-images.json')]
    group = json.loads((folder / 'final/summary.json').read_text())
    regression = json.loads((folder / 'controller-checks.json').read_text())
    paths = ['src/community_demo/community_demo/patrol.py', 'src/community_demo/community_demo/signal_gate.py',
             'src/community_demo/worlds/community.sdf', 'src/community_demo/models/perception/yolo11s-comparison.pt']
    hashes = {p: hashlib.sha256((ROOT / p).read_bytes()).hexdigest() for p in paths}
    before = json.loads((folder / 'before-hashes.json').read_text())
    unchanged = all(hashes[p] == before[p] for p in paths[2:])
    result = {'passed': len(reports) == 3 and group['passed'] and regression['passed'] and unchanged and all(r['passed'] for r in reports),
              'baseline': baseline, 'laps': reports, 'controller_checks': regression,
              'world_and_weights_unchanged': unchanged, 'source_sha256': hashes,
              'limits': ['Three fixed-field laps, six junction passages; sampled physical truth is audit-only',
                         'Red 10 s / green 15 s / yellow 3 s, unchanged; shorter arbitrary cycles unverified',
                         'Already-green arrival with unknown onset still waits for a witnessed transition',
                         'Independent truth phase used to count green cycles; runtime only uses camera recognition',
                         'No claim about new detector accuracy; all photos remain local']}
    (folder / 'summary.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result['passed'] else 1

if __name__ == '__main__':
    raise SystemExit(main())
