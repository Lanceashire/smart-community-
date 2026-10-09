#!/usr/bin/env bash
set -euo pipefail
root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
out_file="${1:-${root_dir}/smart-community-source.zip}"
cd "${root_dir}"
python3 - "${out_file}" <<'PY'
from pathlib import Path
import sys
import zipfile

root = Path.cwd()
output = Path(sys.argv[1]).resolve()
output.parent.mkdir(parents=True, exist_ok=True)
files = [root / name for name in ('README.md', 'NOTICE.md', 'LICENSE', 'requirements-ocr.txt', '.gitignore')]
for directory in ('src', 'tools', 'docker', 'docs', 'licenses', '.devcontainer'):
    files.extend(path for path in (root / directory).rglob('*') if path.is_file())
files = sorted({path for path in files if path != output and '__pycache__' not in path.parts
                and '.git' not in path.parts and path.suffix not in ('.pyc', '.pyo')
                and not any(part.endswith('.egg-info') for part in path.parts)})
with zipfile.ZipFile(output, 'w', compression=zipfile.ZIP_DEFLATED, compresslevel=6) as archive:
    for path in files:
        archive.write(path, path.relative_to(root))
if output.stat().st_size > 150_000_000:
    raise SystemExit('Source archive exceeds the competition 150 MB limit.')
print(f'{output} ({output.stat().st_size / 1_000_000:.2f} MB; {len(files)} files)')
PY
