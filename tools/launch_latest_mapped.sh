#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
map_yaml=$(python3 - <<'PY'
import hashlib,json
from pathlib import Path
reports=sorted(Path('validation').glob('autonomous-mapping-*/final-*/acceptance-summary.json'),key=lambda p:p.stat().st_mtime,reverse=True)
for report in reports:
    data=json.loads(report.read_text())
    if not data.get('passed'):continue
    directory=report.parent/'autonomous-mapping';pgm=directory/'navigation.pgm';yaml=directory/'navigation.yaml'
    if yaml.is_file() and pgm.is_file() and hashlib.sha256(pgm.read_bytes()).hexdigest()==data['mapping']['map_hashes']['navigation.pgm']:
        print(yaml.resolve());break
else:
    # A source checkout includes the exact map from the accepted fixed-field run.
    directory=Path('src/community_demo/maps/accepted-2026-10-06')
    manifest=directory/'manifest.json'
    if not manifest.is_file():raise SystemExit('No verified map: run tools/run_autonomous_mapping.py first.')
    data=json.loads(manifest.read_text())
    for name,expected in data['sha256'].items():
        path=directory/name
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest()!=expected:
            raise SystemExit('Packaged map checksum mismatch: '+name)
    print((directory/'navigation.yaml').resolve())
PY
)
exec bash tools/launch_full.sh "slam:=false" "navigation_map:=$map_yaml" "navigation_config:=$(pwd)/src/community_demo/config/navigation_mapped.yaml" "$@"
