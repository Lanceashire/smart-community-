#!/usr/bin/env python3
"""Finish an interrupted independent image audit of the exact recorded lap.

Does not drive the robot or change acceptance thresholds. Preserve the incomplete
record first; all frozen runtime files must still match before and after.
"""
import hashlib,json,os,subprocess,time
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1];F=ROOT/'validation/four-fixes-2026-10-04'
hashes=json.loads((F/'runtime-hashes.json').read_text())
def frozen():
    return all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h for p,h in hashes.items())
assert frozen(), 'Do not finalize records from a different runtime'
case=F/'sensor-stress';path=case/'lap-001.json';raw=json.loads(path.read_text())
assert raw['checks']['completed'] and raw['checks']['stationary_after_completion']
assert (F/'attempt-05-image-verifier-timeout/lap-001-incomplete-postprocessing.json').exists()
env=os.environ.copy();env['PYTHONPATH']=str(ROOT/'.deps')+os.pathsep+env.get('PYTHONPATH','')
begin=time.monotonic()
p=subprocess.run(['/home/ubuntu/.local/share/aic-vision-venv/bin/python',str(ROOT/'tools/verify_event_images.py'),'--report',str(path),'--output',str(case/'lap-001-images.json')],env=env,capture_output=True,text=True,timeout=90)
(ROOT/'log/four-fixes/recovered-image-verifier.log').write_text(p.stdout+p.stderr)
assert p.returncode==0, 'Independent image OCR still failed'
images=json.loads((case/'lap-001-images.json').read_text())
assert images['passed'] and images['verification_wall_s']<90
events=raw['perception']['events'];checks=raw['checks']
checks['rendered_image_frame_ids']=images['passed']
checks['event_frame_correspondence']=images['passed'] and all(e['same_frame'] and e['matching_box_name_confidence'] and e['image_exists'] and (ROOT/'log/perception'/e['image']).is_file() and hashlib.sha256((ROOT/'log/perception'/e['image']).read_bytes()).hexdigest()==e['sha256'] for e in raw['event_checks'])
terminal=[]
for line in (case/'terminal-events.log').read_text().splitlines():
    start=line.find('{"run_id":')
    if start<0:continue
    try:e=json.loads(line[start:])
    except ValueError:continue
    if (e.get('run_id'),e.get('lap_id'))==(raw['run_id'],raw['lap_id']):terminal.append(e)
checks['terminal_events_exact']=terminal==events
raw.update(passed=all(checks.values()),image_report='lap-001-images.json',
           image_correspondence_method='Independently OCR exact persisted JPEGs; original live physical lap and event records retained',
           raw_image_topic_event_frames_seen=sum(e['annotated_topic_frame_seen'] for e in raw['event_checks']))
assert raw['passed'] and frozen(), 'Do not accept partial checks or changed runtime'
path.write_text(json.dumps(raw,ensure_ascii=False,indent=2)+'\n')
(case/'summary.json').write_text(json.dumps({'passed':True,'laps':1,'reports':['lap-001.json'],'postprocessing_recovered':True,'original_live_suite_exit':1},indent=2)+'\n')
cases=[{'case':name,'passed':json.loads((F/name/'summary.json').read_text())['passed']} for name in ['main','start-plus','start-minus','sensor-stress']]
recovery={'passed':all(c['passed'] for c in cases),'cases':cases,'source_hashes_unchanged':frozen(),'original_live_suite_exit':1,'reason':'Final image worker timed out; exact immutable recorded images independently checked again, same thresholds','verification_wall_s':time.monotonic()-begin,'unique_images':images['unique_images'],'event_count':images['event_count'],'postprocessor_sha256':hashlib.sha256((ROOT/'tools/verify_event_images.py').read_bytes()).hexdigest(),'live_laps_replayed':False}
(F/'suite-processes.json').write_text(json.dumps(recovery,indent=2)+'\n')
print(json.dumps(recovery))
