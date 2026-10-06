#!/usr/bin/env python3
"""Temporarily benchmark archived control sources with the same latest-data injector."""
import hashlib,json,os,signal,subprocess,zipfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
files=['src/community_demo/community_demo/'+n+'.py' for n in ('patrol','perception','localization','person_evidence')]+['src/community_demo/maps/'+n for n in ('roads.pgm','roads.yaml','planning-obstacles.json')]+['tools/launch_full.sh']
f=ROOT/'validation/optimization-2026-10-05/baseline-same-injector';f.mkdir(parents=True,exist_ok=True)
backup={p:(ROOT/p).read_bytes() for p in files};process=None
try:
 with zipfile.ZipFile(ROOT/'validation/optimization-2026-10-04/baseline-source.zip') as z:
  for p in files:(ROOT/p).write_bytes(z.read(p))
 process=subprocess.Popen(['python3','tools/run_four_fixes_acceptance.py','--cases','sensor-stress','--output',str(f.relative_to(ROOT)),'--log-dir','log/optimization-baseline-same-injector'],cwd=ROOT,start_new_session=True)
 code=process.wait(timeout=550)
 if code:raise RuntimeError('Archived baseline failed; do not count as success')
finally:
 if process is not None and process.poll() is None:
  os.killpg(process.pid,signal.SIGINT);process.wait(timeout=35)
 for p,data in backup.items():(ROOT/p).write_bytes(data)
 (f/'source-restoration.json').write_text(json.dumps({'candidate_restored':all((ROOT/p).read_bytes()==data for p,data in backup.items()),'candidate_sha256':{p:hashlib.sha256(data).hexdigest() for p,data in backup.items()},'sensor_injector_sha256':hashlib.sha256((ROOT/'src/community_demo/community_demo/sensor_stress.py').read_bytes()).hexdigest(),'method':'Archived old control, vision, localization and planning map; same current test injector and sensor strengths'},indent=2)+'\n')
