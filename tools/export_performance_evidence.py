#!/usr/bin/env python3
"""Compact numeric audits for GitHub; full sampled records and images remain local."""
import json
from pathlib import Path
root=Path(__file__).resolve().parents[1]/'validation/performance-2026-10-02'
summary=json.loads((root/'summary.json').read_text());assert summary['passed']
out=root/'public';out.mkdir(exist_ok=True)
for group in ['baseline','candidate','start-plus','start-minus']:
    for p in sorted((root/group).glob('lap-*.json')):
        if p.name.endswith('-images.json'):continue
        d=json.loads(p.read_text())
        for key in ['records','performance_frames']:d.pop(key,None)
        for f in d.get('faults',[]):f.pop('samples',None)
        d['raw_evidence_note']='Full sampled pose/latency records in local engineering package; event JPEGs only in local log/'
        (out/f'{group}-{p.name}').write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n')
attempts=[]
for folder in sorted(root.glob('attempt-*')):
    item={'name':folder.name,'included_in_final_acceptance':False}
    for name in ['interruption.json','failure.json']:
        p=folder/name
        if p.exists():
            d=json.loads(p.read_text());item[name]={k:d[k] for k in ['reason','error','max_fresh_position_error_m','diagnostic_truth','diagnostic_estimate','corner_recovery'] if k in d}
    item['completed_laps']=[]
    for p in folder.glob('lap-*.json'):
        if p.name.endswith('-images.json'):continue
        d=json.loads(p.read_text());item['completed_laps'].append({'lap_id':d['lap_id'],'passed':d['passed'],'failed_checks':[k for k,v in d.get('checks',{}).items() if not v]})
    attempts.append(item)
(out/'development-attempts.json').write_text(json.dumps(attempts,ensure_ascii=False,indent=2)+'\n')
print('Exported compact numeric evidence; no images')
