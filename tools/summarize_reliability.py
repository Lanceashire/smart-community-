#!/usr/bin/env python3
"""Aggregate actual laps, camera-angle evidence and physical fault probes."""
import hashlib,json
from pathlib import Path
from summarize_performance import summarize
from summarize_signal_waits import inspect_lap
ROOT=Path(__file__).resolve().parents[1];F=ROOT/'validation/reliability-2026-10-04'
def read(path):return json.loads(path.read_text())
def main():
    group=read(F/'main/summary.json');reports=[];angles=[];public=F/'public';public.mkdir(exist_ok=True)
    for p in sorted((F/'main').glob('lap-*.json')):
        if p.name.endswith('-images.json'):continue
        raw=read(p);metric=summarize(p);gate=inspect_lap(p)
        metric['gate_audit']=gate['gates'];metric['yaw_error_max_deg']=raw['max_fresh_yaw_error_deg'];metric['whole_body_crossings']=raw['whole_body_crossings']
        for view in raw['observation_outcomes']:
            if view['view'].startswith('plate_'):continue
            e=view['feedback'];a=e.get('sample_angles',{});frames=e.get('sample_frames',{})
            angles.append({'lap_id':raw['lap_id'],'view':view['view'],'attempt':view['attempt'],'elapsed_s':view['elapsed_s'],'confirmed':view['confirmed'],'sample_frames':frames,'sample_angles_rad':a,'passed':view['confirmed'] and e.get('two_angles_verified',False) and frames.get('0',0)>=5 and frames.get('1',0)>=3 and abs(a.get('0',0)-a.get('1',0))>=.05})
        # Third lap is also a retry/physical-obstacle probe, never call it a clean timing comparison.
        metric['test_context']={1:'normal autonomous lap',2:'four real node pauses',3:'late-green start plus stationary OCR pause and removable obstacle'}[raw['lap_id']]
        compact={k:v for k,v in raw.items() if k not in ('records','performance_frames')}
        for fault in compact.get('faults',[]):fault.pop('samples',None)
        compact['local_evidence']='Full pose/timing samples in engineering package; all JPEG/PNG and video retained locally only.'
        (public/p.name).write_text(json.dumps(compact,ensure_ascii=False,indent=2)+'\n');reports.append(metric)
    baseline_path=ROOT/'validation/signal-optimization-2026-10-03/final/lap-001.json';baseline=summarize(baseline_path);baseline_raw=read(baseline_path)
    probes={name:read(F/(name+'.json')) for name in ['regression','control-regression','perception-smoke','preflight-pass','launch-supervision','perception-motion','retry-probe','obstacle-probe']}
    frozen=read(F/'runtime-hashes.json');current={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in frozen}
    model=read(F/'model-selection.json')
    checks={'three_laps_physical_and_outputs':group['passed'] and len(reports)==3 and all(r['passed'] for r in reports),'all_fifteen_person_views_two_actual_angles':len(angles)==15 and all(x['passed'] for x in angles),'probes':all(x['passed'] for x in probes.values()),'frozen_runtime_sources':frozen==current,'missing_model_rejected':not read(F/'preflight-missing-model.json')['passed'],'normal_laps_no_extra_green_cycles':all(g['extra_green_cycles']==0 and g['release_within_four_seconds'] for r in reports if r['lap_id']!=2 for g in r['gate_audit'])}
    result={'passed':all(checks.values()),'checks':checks,'baseline_normal_lap':{**baseline,'yaw_error_max_deg':baseline_raw['max_fresh_yaw_error_deg']},'laps':reports,'two_angle_evidence':angles,'probes':{k:{x:y for x,y in v.items() if x not in ('samples','events')} for k,v in probes.items()},'model_selection':model,'runtime_source_sha256':current,'limits':['Fixed supplied 4.2 m field; no arbitrary scene or real robot claim','Detector and held-out metrics unchanged; retraining candidate rejected because plate recall fell','Dual-angle observation and slower corners can increase lap time; reliability changes do not imply faster overall completion','Identity gallery has 16 resident templates and 2 visitor templates; unknown real people are unverified','Green already active with unknown onset waits for a new witnessed transition; real vision outage may intentionally require a later green','Container startup/preflight tested here; clean-machine rebuild recipe is supplied but not independently rebuilt','Two incomplete source assets and team information deferred by user; formal documents remain drafts']}
    (F/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({'passed':result['passed'],'checks':checks,'laps':[{k:r[k] for k in ['lap_id','completion_sim_s','position_error_max_m','yaw_error_max_deg','minimum_post_clearance_m']} for r in reports]},indent=2));return 0 if result['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
