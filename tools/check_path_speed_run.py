"""Audit measured centre tracking, physical speed and manager acceleration."""
import argparse,bisect,json,math,statistics
from pathlib import Path
CORNERS={'top_west':('observe_A_west',-1.7,1.7),'left_middle':('observe_middle',-1.7,.5),'middle_down':('observe_B_east',-.1,.5)}
def summarize(file,trace_file=None,normal=True):
 file=Path(file);raw=json.loads(file.read_text());records=raw['records'];checks={};corners={}
 for name,(route,x,y) in CORNERS.items():
  rows=[r for r in records if r['status'].startswith('GO '+route+' ') and max(abs(r['pose'][0]-x),abs(r['pose'][1]-y))<=.30 and r['speed']>.03]
  speeds=[r['speed']for r in rows];corners[name]={'samples':len(rows),'median_m_s':statistics.median(speeds) if speeds else None,'min_m_s':min(speeds,default=None),'p10_m_s':sorted(speeds)[int(.1*(len(speeds)-1))] if speeds else None}
  if normal:checks[name+'_normal_moving_median_at_least_18cm_s']=bool(rows) and corners[name]['median_m_s']>=.18
 straight=[r for r in records if r['status'].startswith('GO right_north ') and -.30<=r['pose'][1]<=.60]
 centre={'samples':len(straight),'max_error_m':max((abs(r['pose'][0]-1.1)for r in straight),default=None),'median_speed_m_s':statistics.median([r['speed']for r in straight]) if straight else None}
 checks['actual_long_straight_centre_error_under_2cm']=bool(straight) and centre['max_error_m']<=.02
 if normal:checks['normal_long_straight_median_at_least_245cm_s']=bool(straight) and centre['median_speed_m_s']>=.245
 control={}
 if trace_file:
  trace=json.loads(Path(trace_file).read_text());start,end=records[0]['t'],records[-1]['t'];planner=[r for r in trace['planner']if r['run_id']==raw['run_id'] and r['lap_id']==raw['lap_id']]
  navigation=[r for r in trace['navigation']if start<=r['stamp']<=end];manager=[r for r in trace['manager']if start<=r['stamp']<=end]
  aa=[];nonpositive=0
  for a,b in zip(manager,manager[1:]):
   dt=b['stamp']-a['stamp']
   if dt<=0:nonpositive+=1;continue
   aa.append((abs(b['linear']-a['linear'])/dt,abs(b['angular']-a['angular'])/dt))
  control={'planner_samples':len(planner),'navigation_samples':len(navigation),'manager_samples':len(manager),'manager_nonpositive_stamp_intervals':nonpositive,
    'navigation_max_linear_m_s':max((abs(r['linear'])for r in navigation),default=None),'manager_max_linear_m_s':max((abs(r['linear'])for r in manager),default=None),
    'curved_requested_max_angular_rad_s':max((abs(r['angular'])for r in planner if r['curved']),default=None),
    'manager_max_angular_rad_s':max((abs(r['angular'])for r in manager),default=None),
    'manager_max_linear_accel_m_s2':max((a for a,b in aa),default=None),'manager_max_angular_accel_rad_s2':max((b for a,b in aa),default=None),
    'quality_states':{s:sum(r.get('localization_quality')==s for r in planner) for s in ('tracking','degraded','lost')}}
  checks['trace_has_this_lap_and_all_command_stages']=bool(planner and navigation and manager and aa)
  checks['navigation_and_manager_linear_limits']=bool(navigation and manager) and max(control['navigation_max_linear_m_s'],control['manager_max_linear_m_s'])<=.26+1e-8
  checks['curved_angular_limit']=control['curved_requested_max_angular_rad_s'] is not None and control['curved_requested_max_angular_rad_s']<=.65+1e-8
  checks['manager_angular_limit']=bool(manager) and control['manager_max_angular_rad_s']<=.70+1e-8
  checks['manager_linear_acceleration_limit']=bool(aa) and control['manager_max_linear_accel_m_s2']<=.50+1e-7
  checks['manager_angular_acceleration_limit']=bool(aa) and control['manager_max_angular_accel_rad_s2']<=1.20+1e-7
 return {'passed':all(checks.values()),'checks':checks,'run_id':raw['run_id'],'lap_id':raw['lap_id'],'normal_speed_gates':normal,'corners':corners,'return_straight':centre,'control':control,
   'scope':'Ground-truth speed and centre error are read-only evaluation. Same 60cm corner windows, GO samples over 3cm/s; required stops excluded for both versions. Stress/dim/dynamic still satisfy centre/safety/command limits; minimum normal cruise-speed gates apply only to normal mapping, saved-map, packaged-map and repeat laps.'}
def main():
 p=argparse.ArgumentParser();p.add_argument('--lap',required=True);p.add_argument('--trace');p.add_argument('--degraded-case',action='store_true');p.add_argument('--output',required=True);a=p.parse_args();r=summarize(a.lap,a.trace,not a.degraded_case);Path(a.output).write_text(json.dumps(r,indent=2)+'\n');print(json.dumps({'passed':r['passed'],'failed':[k for k,v in r['checks'].items()if not v],'corners':r['corners'],'return_straight':r['return_straight'],'control':r['control']},indent=2));raise SystemExit(0 if r['passed']else 1)
if __name__=='__main__':main()
