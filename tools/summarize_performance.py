#!/usr/bin/env python3
"""Derive measured timing, latency and error from unchanged physical audit records."""
import argparse,json,statistics
from collections import defaultdict
from pathlib import Path

def percentile(values,p):
    values=sorted(v for v in values if v is not None)
    return values[min(len(values)-1,int((len(values)-1)*p))] if values else None

def summarize(path):
    d=json.loads(path.read_text());r=d['records'];durations=defaultdict(float)
    for a,b in zip(r,r[1:]):
        state=a['status'].split(' ',1)[0]
        durations[state]+=b['t']-a['t']
    # On a live restart the first truth frame can retain the preceding lap's
    # status. Only a COMPLETE after this lap's active states ends its timing.
    seen_active=False;finish=len(r)-1
    for i,x in enumerate(r):
        if not x['status'].startswith('COMPLETE'):seen_active=True
        elif seen_active:finish=i;break
    active=r[:finish+1];frames=d.get('performance_frames',[])
    elapsed=active[-1]['t']-active[0]['t'];wall=active[-1]['wall_s']-active[0]['wall_s']
    if not seen_active or wall<=0 or elapsed<=0:
        raise ValueError('No positive current-lap activity interval: '+str(path))
    e=[x['fresh_pose_error'][0] for x in active if x['fresh_pose_error']]
    return {'source':str(path),'passed':d['passed'],'mode':d['mode'],'run_id':d['run_id'],'lap_id':d['lap_id'],
      'completion_sim_s':round(elapsed,3),'completion_wall_s':round(wall,3),'real_time_factor':round(elapsed/wall,3),
      'state_sim_s':{k:round(v,3) for k,v in durations.items()},'position_error_max_m':max(e),
      'position_error_p95_m':percentile(e,.95),'position_error_rmse_m':(statistics.mean(v*v for v in e))**.5,
      'processing_ms_p50':percentile([f['processing_ms'] for f in frames],.5),'processing_ms_p95':percentile([f['processing_ms'] for f in frames],.95),
      'image_age_sim_s_p95':percentile([f['image_age_sim_s'] for f in frames],.95),
      'perception_hz_wall':round((len(frames)-1)/(frames[-1]['wall']-frames[0]['wall']),3) if len(frames)>1 else None,
      'observations':[{k:x[k] for k in ['view','elapsed_s','confirmed','attempt']} for x in d['perception'].get('observation_outcomes',[])],
      'minimum_post_clearance_m':d.get('minimum_post_clearance_m'),'counts':d['counts'],'plates':d['plates'],'checks':d['checks'],'road_violation_samples':d['road_violation_samples']}

def main():
    p=argparse.ArgumentParser();p.add_argument('folders',nargs='+');p.add_argument('--output',required=True);a=p.parse_args()
    reports=[summarize(path) for f in a.folders for path in sorted(Path(f).glob('lap-*.json')) if not path.name.endswith('-images.json')]
    result={'passed':bool(reports) and all(r['passed'] for r in reports),'laps':reports,'note':'Completion excludes post-finish audit hold. Runtime processing includes detector, OCR and overlay but not every publishing/disk stage. Wall Hz and simulation time are distinct.'}
    Path(a.output).write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n');print(json.dumps(result,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
