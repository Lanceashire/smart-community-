#!/usr/bin/env python3
"""Combine measured laps, actual terminal events and independent saved-image checks."""
import json,math
from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
from community_demo.road_rules import footprint
root=Path(__file__).resolve().parents[1];folder=root/'validation/patrol-cycle-review/final'
reports=[json.loads((folder/f'lap-{i:03d}.json').read_text()) for i in (1,2)]
terminal={}
for line in (root/'log/perception-terminal.log').read_text().splitlines():
 start=line.find('{"run_id":')
 if start<0:continue
 try:e=json.loads(line[start:])
 except ValueError:continue
 terminal.setdefault((e['run_id'],e['lap_id'],e['sequence']),[]).append(e)
results=[]
for d in reports:
 events=d['perception']['events'];terminal_matches=all(terminal.get((e['run_id'],e['lap_id'],e['sequence']))==[e] for e in events)
 crossing=[];last=[None,None];red_straddles=0
 for r in d['records']:
  x,y,yaw=r['pose'];points=footprint(x,y,yaw)
  distances=[(min(a for a,b in points)-.3,max(a for a,b in points)-.3),(min(b for a,b in points)+.51,max(b for a,b in points)+.51)]
  for i,near in enumerate([abs(y-1.7)<.3,abs(x+.1)<.3]):
   current=distances[i]
   if not near:last[i]=None;continue
   if current[0]<0<current[1] and r['phase']=='red':red_straddles+=1
   if last[i]:
    for j,name in enumerate(['front_entered','rear_cleared']):
     if last[i][j]>=0 and current[j]<0:crossing.append({'gate':i+1,'edge':name,'time':r['t'],'phase':r['phase']})
   last[i]=current
 rear_green=len(crossing)==4 and all(c['phase']=='green' for c in crossing)
 images=json.loads((folder/d['image_report']).read_text())
 supplemental={'terminal_events_exact':terminal_matches,'rendered_event_images':images['passed'],'front_and_rear_green':rear_green,'red_straddle_samples_zero':red_straddles==0}
 faults=[]
 for f in d['faults']:
  stopped=[s['elapsed'] for s in f['samples'] if abs(s['command'][0])<.001 and abs(s['command'][1])<.001 and s['speed']<.015]
  faults.append({'node':f['node'],'pause_s':f['duration_wall_s'],'first_observed_stop_s':min(stopped) if stopped else None,'settled_speed_max':f['settled_speed_max'],'passed':f['passed']})
 result={k:d[k] for k in ['run_id','lap_id','mode','sim_elapsed_s','physical_distance_m','counts','plates','max_fresh_position_error_m','max_fresh_yaw_error_deg','road_violation_samples','red_waits','crossings','samples']}
 result.update(passed=d['passed'] and all(supplemental.values()),checks={**d['checks'],**supplemental},waypoint_count=len(d['waypoints']),observation_count=len(d['observations']),events=len(events),unique_event_images=images['unique_images'],raw_topic_events_seen=d['raw_image_topic_event_frames_seen'],whole_body_crossings=crossing,faults=faults)
 results.append(result)
probe=json.loads((folder/'lower-red-probe.json').read_text());lifecycle=json.loads((root/'validation/patrol-cycle-review/lifecycle-replay.json').read_text())
summary={'review_time':datetime.now(ZoneInfo('Asia/Shanghai')).isoformat(),'passed':all(d['passed'] for d in results) and probe['passed'] and lifecycle['passed'],'laps':results,'lower_red_probe_passed':probe['passed'],'lifecycle_replay_passed':lifecycle['passed'],'prior_run':{'normal_laps_passed':2,'fault_lap_timed_out':True,'failure_record':'../before-recovery-fix.json'},'image_validation':'Events match actual terminal JSON, same-frame detector boxes/names/confidences, and OCR-read Frame numbers from the saved JPEGs. Best-effort raw topic reception is diagnostic.','limits':['One fixed supplied-gallery scene and known initial pose','Short node pauses and controlled red wait; not process crashes, GPU resets or arbitrary dynamic obstacles','No proof of unseen-person or real-camera recognition','Two original source images remain deferred by user'],'photos':'Only in local log/; never uploaded'}
summary['explicit_red_line_checks_passed']=all(d['checks']['red_straddle_samples_zero'] and d['checks']['two_green_crossings'] for d in results)
summary['strict_all_edges_green_passed']=all(d['checks']['front_and_rear_green'] for d in results)
summary['open_observation']='Normal lap lower gate: front crossed on green at t=584.10; rear cleared on yellow at t=585.35. No red crossing or red straddling was observed. The supplied text does not separately define yellow clearance; report this boundary explicitly rather than claim every body edge crossed only on green.'
(folder/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n');print(json.dumps(summary,ensure_ascii=False,indent=2));raise SystemExit(0 if summary['passed'] else 1)
