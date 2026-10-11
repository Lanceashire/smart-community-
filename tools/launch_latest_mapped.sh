#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
map_yaml=$(python3 - <<'PY'
import hashlib,json
from pathlib import Path
task_hash=hashlib.sha256(Path('src/community_demo/config/patrol_task.json').read_bytes()).hexdigest()
reports=sorted(Path('validation/unified-trajectory-2026-10-11').glob('acceptance*.json'),key=lambda p:p.stat().st_mtime,reverse=True)
for report in reports:
    data=json.loads(report.read_text())
    if not data.get('passed') or data.get('task_sha256')!=task_hash:continue
    directory=Path(data['mapping']['directory']);pgm=directory/'navigation.pgm';yaml=directory/'navigation.yaml'
    if yaml.is_file() and pgm.is_file() and hashlib.sha256(pgm.read_bytes()).hexdigest()==data['mapping']['map_hashes']['navigation.pgm']:
        print(yaml.resolve());break
else:
    # The checked map travels with the source archive, tied to the exact task.
    directory=Path('docs/accepted-navigation-2026-10-11')
    manifest=directory/'manifest.json'
    if not manifest.is_file():raise SystemExit('No map verified for the current task: run tools/run_autonomous_mapping.py first.')
    data=json.loads(manifest.read_text())
    if data.get('task_sha256')!=task_hash:raise SystemExit('Packaged map belongs to a different task configuration; rebuild the map.')
    for name,expected in data['sha256'].items():
        path=directory/name
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest()!=expected:
            raise SystemExit('Packaged map checksum mismatch: '+name)
    print((directory/'navigation.yaml').resolve())
PY
)
exec bash tools/launch_full.sh "slam:=false" "navigation_map:=$map_yaml" "navigation_config:=$(pwd)/src/community_demo/config/navigation_mapped.yaml" "$@"
