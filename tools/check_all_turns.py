"""Read-only complete-lap motion audit; keep observation and traffic waits separate."""
import argparse, hashlib, json, math
from pathlib import Path

CORNERS = {'top_west': (-1.7, 1.7), 'left_middle': (-1.7, .5),
           'middle_down': (-.1, .5), 'lower_east': (-.1, -1.7),
           'plate_entry': (1.1, -1.7), 'return_start': (1.1, 1.7)}

def motion(rows):
    signed = variation = stationary_rotation = 0.
    switches = []; pauses = []; pause_start = None
    for a,b in zip(rows,rows[1:]):
        dt=b['t']-a['t']
        if not 0 < dt <= .2: continue
        delta=math.atan2(math.sin(b['pose'][2]-a['pose'][2]),math.cos(b['pose'][2]-a['pose'][2]))
        signed+=delta; variation+=abs(delta)
        if a['speed'] < .02 and abs(delta/dt) > .05: stationary_rotation+=dt
        sign=1 if delta/dt>.1 else (-1 if delta/dt<-.1 else 0)
        if sign and (not switches or switches[-1]!=sign): switches.append(sign)
        if a['speed'] < .012:
            if pause_start is None: pause_start=a['wall_s']
        elif pause_start is not None:
            if a['wall_s']-pause_start>=.25: pauses.append(a['wall_s']-pause_start)
            pause_start=None
    return {'duration_wall_s': rows[-1]['wall_s']-rows[0]['wall_s'] if rows else 0.,
            'samples':len(rows),'net_heading_deg':math.degrees(signed),'heading_variation_deg':math.degrees(variation),
            'excess_heading_variation_deg':math.degrees(max(0.,variation-abs(signed))),
            'direction_changes':max(0,len(switches)-1),'stationary_rotation_sim_s':stationary_rotation,
            'internal_stop_count':len(pauses),'internal_stopped_wall_s':sum(pauses)}

def summarize(path):
    path=Path(path);d=json.loads(path.read_text());records=d['records'];corners={}
    first_plate=next(r['t'] for r in records if r['status'].startswith('OBSERVE plate_1'))
    for name,(x,y) in CORNERS.items():
        rows=[r for r in records if max(abs(r['pose'][0]-x),abs(r['pose'][1]-y))<=.30]
        if name=='plate_entry':rows=[r for r in rows if r['t']<first_plate]
        if name=='return_start':rows=[r for r in rows if r['t']>first_plate and not r['status'].startswith('COMPLETE')]
        corners[name]=motion(rows)
    align={}
    for name in ('observe_A_west','observe_middle','observe_B_east','observe_plate_1','observe_plate_2','observe_plate_3'):
        rows=[r for r in records if r['status'].startswith('ALIGN_CAMERA '+name)]
        align[name]=motion(rows)
    return {'run_id':d['run_id'],'lap_id':d['lap_id'],'passed':d['passed'],
            'lap_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'corners':corners,'camera_alignment':align,
            'stationary_rotation_sim_s_total':sum(x['stationary_rotation_sim_s'] for x in corners.values()),
            'road_violations':d['road_violation_samples'],'red_straddles':d['red_straddle_samples'],
            'scope':'Same fixed 60cm corner windows from actual Gazebo pose samples; plate-entry ends before sampling, return-start excludes initial departure and completion tail. Includes waypoint planning stops, excludes required observation/light waits.'}

def main():
    p=argparse.ArgumentParser();p.add_argument('--laps',nargs='+',required=True);p.add_argument('--output',required=True);a=p.parse_args()
    rows=[summarize(x) for x in a.laps];out=Path(a.output);out.parent.mkdir(parents=True,exist_ok=True)
    result={'passed':all(x['passed'] and x['road_violations']==x['red_straddles']==0 for x in rows),'laps':rows}
    out.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({'passed':result['passed'],'laps':[{'run_id':r['run_id'],'stationary_rotation_sim_s_total':r['stationary_rotation_sim_s_total'],'corners':r['corners']} for r in rows]},indent=2))
if __name__=='__main__':main()
