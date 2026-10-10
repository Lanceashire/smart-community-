#!/usr/bin/env python3
"""Independent actual-motion audit: distinguish a rolling turn from oscillation."""
import argparse,json,math
from pathlib import Path


def metrics(path):
    lap=json.loads(Path(path).read_text());events=lap['status_changes']
    start=next(x['wall_s']for x in events if x['status'].startswith('REACHED lower_south'))
    end=next(x['wall_s']for x in events if x['status'].startswith('REACHED bottom_east'))
    rows=[r for r in lap['records']if start<=r['wall_s']<=end]
    signed=absolute=0.;directions=[];previous=None;pause_start=None;pauses=[]
    for r in rows:
        if previous and r['speed']>.025 and previous['speed']>.025 and 0<r['t']-previous['t']<=.2:
            dt=r['t']-previous['t'];delta=math.atan2(math.sin(r['pose'][2]-previous['pose'][2]),math.cos(r['pose'][2]-previous['pose'][2]))
            signed+=delta;absolute+=abs(delta);rate=delta/dt
            sign=1 if rate>.1 else (-1 if rate<-.1 else 0)
            if sign and(not directions or sign!=directions[-1]):directions.append(sign)
        if start+.7<=r['wall_s']<=end-.2 and r['speed']<.012:
            if pause_start is None:pause_start=r['wall_s']
        elif pause_start is not None:
            if r['wall_s']-pause_start>=.25:pauses.append(r['wall_s']-pause_start)
            pause_start=None
        previous=r
    return {'run_id':lap['run_id'],'duration_wall_s':end-start,'internal_stop_count':len(pauses),
            'internal_stopped_wall_s':sum(pauses),'moving_heading_change_deg':math.degrees(signed),
            'moving_heading_variation_deg':math.degrees(absolute),
            'excess_moving_heading_variation_deg':math.degrees(max(0.,absolute-abs(signed))),
            'moving_turn_direction_changes':max(0,len(directions)-1),
            'road_recovery_episodes':sum(start<=x['wall_s']<=end and x['status'].startswith('RECOVER_ROAD')for x in events),
            'road_violations':lap['road_violation_samples'],'red_straddles':lap['red_straddle_samples'],
            'minimum_post_clearance_m':lap['minimum_post_clearance_m'],'passed':lap['passed'],
            'completed_sim_s':next(r['t']for r in lap['records']if r['status'].startswith('COMPLETE'))-lap['records'][0]['t']}


def main():
    p=argparse.ArgumentParser();p.add_argument('--run',required=True);p.add_argument('--baseline',required=True);p.add_argument('--output',required=True);a=p.parse_args()
    baseline=metrics(a.baseline);cases={x.parent.name:metrics(x)for x in Path(a.run).glob('*/lap-001.json')};assert len(cases)==3
    checks={}
    for name,row in cases.items():
        checks.update({name+'_complete':row['passed'],name+'_no_internal_turn_stop':row['internal_stop_count']==0,
                       name+'_rolls_through_right_angle':75<=row['moving_heading_change_deg']<=120,
                       name+'_no_oscillatory_heading':row['excess_moving_heading_variation_deg']<=15,
                       name+'_at_most_two_direction_changes':row['moving_turn_direction_changes']<=2,
                       name+'_road_and_red_compliant':row['road_violations']==0 and row['red_straddles']==0,
                       name+'_no_road_recovery':row['road_recovery_episodes']==0,
                       name+'_post_clearance_retains_5cm':row['minimum_post_clearance_m']>=.05})
    checks['baseline_has_stopped_turns']=baseline['internal_stop_count']>=1
    result={'passed':all(checks.values()),'checks':checks,'failed':[k for k,v in checks.items()if not v],'baseline':baseline,'cases':cases,
            'scope':'Read-only Gazebo source-stamped motion. A continuous 90 degree turn necessarily adds moving heading change; excess absolute variation and actual internal pauses measure smoothness. Safety/perception remain in independent full-lap audit.'}
    out=Path(a.output);out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2));return 0 if result['passed']else 1
if __name__=='__main__':raise SystemExit(main())
