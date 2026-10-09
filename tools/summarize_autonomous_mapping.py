#!/usr/bin/env python3
"""Aggregate independent map provenance, physical laps and unchanged safety gates."""
import argparse,ast,hashlib,json
from pathlib import Path
import numpy as np
from summarize_performance import summarize
from summarize_four_fixes import gate_review,blackout_error_bound
ROOT=Path(__file__).resolve().parents[1]
def read(p):return json.loads(p.read_text())
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
 p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--regressions',required=True);a=p.parse_args()
 out=ROOT/a.output;reg=ROOT/a.regressions;checks={};rows=[]
 pipeline=read(out/'pipeline.json');checks['three_stage_pipeline']=pipeline['passed'] and [x['case'] for x in pipeline['cases']]==['autonomous-mapping','mapped-navigation','mapped-dynamic']
 hashes=read(out/'runtime-hashes.json')
 sources=[p for p in (ROOT/'src').rglob('*') if p.is_file() and '__pycache__' not in str(p)]+[p for p in (ROOT/'tools').iterdir() if p.is_file() and p.suffix in ('.py','.sh','.bash')]
 checks['source_inventory_frozen']=set(hashes)=={str(p.relative_to(ROOT)) for p in sources}
 evaluator='tools/summarize_autonomous_mapping.py'
 checks['all_execution_source_hashes_frozen']=all((ROOT/k).is_file() and digest(ROOT/k)==v for k,v in hashes.items() if k!=evaluator)
 old_evaluator=out/'frozen-summarize-autonomous-mapping.py';analysis_change=digest(ROOT/evaluator)!=hashes[evaluator]
 checks['post_run_evaluator_change_disclosed']=not analysis_change or (old_evaluator.is_file() and digest(old_evaluator)==hashes[evaluator])
 analysis={'file':evaluator,'snapshot_sha256':hashes[evaluator],'current_sha256':digest(ROOT/evaluator),'changed_after_laps':analysis_change,'archived_snapshot':str(old_evaluator) if analysis_change else None,'reason':'Read-only stress observer omitted from the post-run allowlist; execution source files and all production safety gates unchanged.'}
 mapping=read(out/'autonomous-mapping/mapping-evidence.json')
 checks.update({'mapping_'+k:v for k,v in mapping['checks'].items()})
 checks['all_generated_map_hashes_match']=all(digest(out/'autonomous-mapping'/k)==v for k,v in mapping['map_hashes'].items())
 pgmhash=mapping['map_hashes']['navigation.pgm']
 checks['unknown_cells_never_declared_free']=mapping['planning_grid']['unknown_is_free'] is False
 hits=np.load(out/'autonomous-mapping/occupancy.npz')['laser_hit_counts'];xy=(np.argwhere(hits>=3)[:,::-1]+.5)*.02-2.1
 # Known physical coordinates are used ONLY here as evaluation references.
 posts=read(ROOT/'src/community_demo/maps/planning-obstacles.json')['posts']
 post_errors=[{'name':post['name'],'nearest_confirmed_laser_cell_m':float(np.min(np.linalg.norm(xy-[post['x'],post['y']],axis=1))) if len(xy) else None} for post in posts]
 checks['all_four_posts_have_measured_cells_nearby']=all(x['nearest_confirmed_laser_cell_m'] is not None and x['nearest_confirmed_laser_cell_m']<=.06 for x in post_errors)
 rp=read(reg/'pipeline.json');checks['three_live_regressions']=rp['passed'] and {x['case'] for x in rp['cases']}=={'moving-faults','sensor-stress','dim-light'}
 checks['regressions_use_same_sources']=read(reg/'runtime-hashes.json')==hashes
 checks['all_case_sources_frozen_during_execution']=all(x['source_frozen'] for x in pipeline['cases']+rp['cases'])
 tree=ast.parse((ROOT/'src/community_demo/community_demo/sensor_stress.py').read_text());cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='SensorStress');methods={n.name:n for n in cls.body if isinstance(n,ast.FunctionDef)}
 callback=methods['on_truth'].body
 checks['stress_truth_callback_diagnostics_only']=len(callback)==1 and isinstance(callback[0],ast.Assign) and len(callback[0].targets)==1 and ast.unparse(callback[0].targets[0])=='self.speed' and not any(isinstance(n,ast.Attribute) and n.attr=='speed' for n in ast.walk(methods['receive']))
 groups=[(out,x) for x in ['autonomous-mapping','mapped-navigation','mapped-dynamic']]+[(reg,x) for x in ['moving-faults','sensor-stress','dim-light']]
 for parent,name in groups:
  case=parent/name;raw=read(case/'lap-001.json');m=summarize(case/'lap-001.json');gates=[gate_review(raw,x) for x in raw['signal_decisions']]
  checks.update({name+'_'+k:v for k,v in raw['checks'].items()})
  checks[name+'_core']=read(case/'summary.json')['passed'] and raw['passed']
  checks[name+'_actual_lamp_integrity']=read(case/'signal-integrity.json')['passed']
  checks[name+'_position_error_bound']=m['position_error_max_m']<=(.12 if name=='sensor-stress' else .06)
  checks[name+'_post_clearance']=m['minimum_post_clearance_m']>=.05
  checks[name+'_vision_at_least_8hz']=m['perception_hz_wall']>=8.
  checks[name+'_image_age_p95_under_150ms']=m['image_age_sim_s_p95']<=.15+1e-9
  checks[name+'_green_budget_and_no_extra_cycle']=len(gates)==2 and all(g['prediction_is_conservative'] and g['whole_body_budget_met'] and g['already_green_no_long_wait'] for g in gates)
  subs=read(case/'summary.json')['evaluation_only_subscribers']
  checks[name+'_truth_audit_only']=all(set(v)<= ({'patrol_cycle_audit','independent_signal_integrity','dynamic_interference_probe'}|({'sensor_stress_acceptance'} if name=='sensor-stress' else set())) for v in subs.values())
  if name.startswith('mapped-'):
   mv=read(case/'mapped-navigation-evidence.json');checks.update({name+'_map_'+k:v for k,v in mv['checks'].items()})
   checks[name+'_uses_this_generated_map']=mv['pgm_sha256']==pgmhash
  if name=='sensor-stress':
   stress=raw['stress_statistics'][-1];checks['stress_blackout_recovered']=stress['blackout_finished'] and stress['blackout_stop_passed'];checks['stress_stale_error_bounded']=blackout_error_bound(raw,stress)['passed']
  rows.append({'case':name,'passed':raw['passed'],'completion_sim_s':m['completion_sim_s'],'position_error_max_cm':m['position_error_max_m']*100,'position_error_p95_cm':m['position_error_p95_m']*100,'post_clearance_min_cm':m['minimum_post_clearance_m']*100,'perception_hz_wall':m['perception_hz_wall'],'image_age_p95_s':m['image_age_sim_s_p95'],'road_violation_samples':raw['road_violation_samples'],'red_straddle_samples':raw['red_straddle_samples'],'counts':m['counts'],'plates':m['plates'],'gates':gates})
 dynamic=read(out/'mapped-dynamic/dynamic-interference.json');checks.update({'dynamic_'+k:v for k,v in dynamic['checks'].items()});checks['dynamic_clearance_at_least_2cm']=dynamic['minimum_body_clearance_m']>=.02
 cm=np.load(out/'mapped-dynamic/costmaps.npz');t=cm['stamps'];row=int(np.floor((dynamic['obstacle_y']+2.1)/.02));col=int(np.floor((1.1+2.1)/.02));v=cm['grids'][:,row,col]
 before=(t>=dynamic['start_sim']-5)&(t<dynamic['start_sim']);during=(t>=dynamic['blocked_sim'])&(t<dynamic['clear_sim']);after=t>dynamic['resume_sim']
 checks['dynamic_cell_free_before_entry']=bool(before.any()) and bool((v[before]==0).all())
 checks['dynamic_obstacle_actually_marked_in_costmap']=bool(during.any()) and bool((v[during]>=99).any())
 checks['dynamic_cell_cleared_after_exit']=bool(after.any()) and bool((v[after]==0).all())
 regression=out.parent
 for name in ['sensor-regression.json','grid-regression-corrected.json','control-regression-final.json','boundary-regression-final.json']:
  r=read(regression/name);checks.update({name+'_'+k:v for k,v in r['checks'].items()})
 result={'passed':all(checks.values()),'checks':checks,'failed_checks':[k for k,v in checks.items() if not v],'runtime_files':len(hashes),'post_run_analysis_review':analysis,'laps':rows,'mapping':{k:v for k,v in mapping.items() if k!='map_statistics'},'map_growth':{'updates':len(mapping['map_statistics']),'distinct_grids':len({x['sha256'] for x in mapping['map_statistics']}),'first_known_cells':mapping['map_statistics'][0]['known_cells'],'last_known_cells':mapping['map_statistics'][-1]['known_cells']},'post_map_evaluation':post_errors,'dynamic':{k:dynamic[k] for k in ['method','capability','minimum_body_clearance_m','hold_speed_max_m_s','start_sim','blocked_sim','clear_sim','resume_sim']},'scope':['Supplied fixed field and known start; autonomous route mapping with supplied lane prior, not frontier exploration.','Map combines live SLAM grid and repeated real laser endpoints registered with camera/wheel/IMU TF. No ground-truth pose used by controller or map collector.','Gazebo ideal IMU heading; real hardware calibration, IMU noise and arbitrary-pose relocalization untested.','Obstacle entry/stop/wait/clear/replan/resume tested; persistent-obstacle active bypass unproven.','Two independent failures and map quality/validator failures retained under candidate-01 through candidate-04; excluded from final results.','Mapping video is a live map renderer plus actual robot camera, not a screen recording of RViz. Formal combined submission video remains separate.']}
 (out/'acceptance-summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n');print(json.dumps({'passed':result['passed'],'checks':len(checks),'failed':result['failed_checks'],'laps':rows},ensure_ascii=False,indent=2));return 0 if result['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
