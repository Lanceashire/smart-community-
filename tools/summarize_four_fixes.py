#!/usr/bin/env python3
"""Strict acceptance: no skipped conditions, no failed development runs counted."""
import argparse,hashlib,json,math
from pathlib import Path
from summarize_performance import summarize

ROOT=Path(__file__).resolve().parents[1];F=ROOT/'validation/four-fixes-2026-10-04'

def blackout_error_bound(raw,stress):
    """Stale auditor estimate is held: ||p-e|| <= last error + displacement.

    Estimate headers are paired to physical source times, with no extrapolation
    through the blackout. The final exact pre-gap estimate is held until new
    wheel data arrives. Missing samples are never counted as zero error.
    """
    last=None;previous=None;bounds=[];stale=0;gaps=[]
    start=stress['blackout_start_sim_s'];end=stress['blackout_end_sim_s']
    for r in raw['records']:
        if r['fresh_pose_error'] is not None:last=r
        if start<=r['t']<=end:
            if previous is not None:gaps.append(r['wall_s']-previous['wall_s'])
            if last is None:return {'passed':False,'reason':'No pre-blackout estimate'}
            bound=(r['fresh_pose_error'][0] if r['fresh_pose_error'] is not None else
                   last['fresh_pose_error'][0]+math.hypot(r['pose'][0]-last['pose'][0],r['pose'][1]-last['pose'][1]))
            stale+=r['fresh_pose_error'] is None
            bounds.append(bound)
        previous=r
    # Physical records are dense; the source gap cannot hide an estimate
    # inside a missing recorded interval. The held estimate is bounded below.
    return {'passed':len(bounds)>=20 and bool(gaps) and max(gaps)<.7 and max(bounds)<=.12,'samples':len(bounds),'stale_samples':stale,
            'maximum_truth_sample_gap_wall_s':max(gaps),
            'maximum_position_error_upper_bound_cm':max(bounds)*100,
            'method':'Fresh errors measured; held stale estimate bounded by last measured error plus independent actual displacement, triangle inequality; not zero-filled'}

def gate_review(raw,decision):
    records=raw['records'];gate=decision['gate']
    approach='top_approach' if gate=='top_stop' else 'lower_approach'
    start=next(i for i,r in enumerate(records) if r['status'].startswith(('GO '+approach,'PLAN '+approach)))
    end=next(i for i in range(start,len(records)) if records[i]['status'].startswith('REACHED '+gate))
    waits=[r for r in records[start:end+1] if r['status'].startswith('STOP '+gate)]
    green_end=next((r['t'] for r in records[end+1:] if r['phase'] in ('red','yellow')),None)
    wait=records[end]['t']-waits[0]['t'] if waits else 0.
    already_green=records[start]['phase']=='green' and all(r['phase'] not in ('red','yellow') for r in records[start:end+1])
    left=decision['remaining_lower_s']
    return {'gate':gate,'release_sim_s':decision['stamp'],'stop_wait_sim_s':round(wait,3),
        'approach_started_green':records[start]['phase']=='green',
        'passed_same_already_active_green':already_green,
        'remaining_lower_s':left,'actual_remaining_to_next_non_green_s':None if green_end is None else green_end-decision['stamp'],
        'prediction_is_conservative':green_end is not None and left<=green_end-decision['stamp']+.1,
        'whole_body_budget_met':left>=decision['crossing_budget_s'],
        'already_green_no_long_wait':not already_green or wait<=2.,
        'camera_edge_count':decision['camera_edge_count']}

def main():
    global F
    parser=argparse.ArgumentParser();parser.add_argument('--output',default='validation/four-fixes-2026-10-04');parser.add_argument('--main-laps',type=int,default=3);a=parser.parse_args();F=ROOT/a.output
    rows=[];native=[];angles=[];direct=[];checks={};stress=None
    for case,count in [('main',a.main_laps),('start-plus',1),('start-minus',1),('sensor-stress',1)]:
        group=json.loads((F/case/'summary.json').read_text())
        paths=[p for p in sorted((F/case).glob('lap-*.json')) if not p.name.endswith('-images.json')]
        checks[case+'_complete']=group['passed'] and len(paths)==count
        for path in paths:
            raw=json.loads(path.read_text());metrics=summarize(path)
            gates=[gate_review(raw,d) for d in raw['signal_decisions']]
            checks[case+'_two_decisions_'+str(raw['lap_id'])]=len(gates)==2
            completed=next((i for i,r in enumerate(raw['records']) if r['status'].startswith('COMPLETE')),None)
            checks[case+'_completion_status_latched_'+str(raw['lap_id'])]=(completed is not None and all(r['status'].startswith('COMPLETE') for r in raw['records'][completed:]))
            direct.extend(g for g in gates if g['passed_same_already_active_green'])
            checks[case+'_green_safety_'+str(raw['lap_id'])]=all(g['prediction_is_conservative'] and g['whole_body_budget_met'] and g['already_green_no_long_wait'] for g in gates)
            diagnostics=raw.get('signal_diagnostics',[])
            native.extend(diagnostics)
            for view in raw['observation_outcomes']:
                if view['view'].startswith('plate_'):continue
                f=view['feedback'];sample_angles=f.get('sample_angles',{});n=f.get('sample_frames',{})
                angles.append({'case':case,'lap':raw['lap_id'],'view':view['view'],'passed':view['confirmed'] and f.get('two_angles_verified') and n.get('0',0)>=5 and n.get('1',0)>=3 and abs(sample_angles.get('0',0)-sample_angles.get('1',0))>=.05})
            b=next(v for v in raw['observation_outcomes'] if v['view']=='B_north')
            if case=='main':checks['B_north_no_retry_lap_'+str(raw['lap_id'])]=b['attempt']==0 and b['elapsed_s']<=6.5
            max_error=.12 if case=='sensor-stress' else .10 if case.startswith('start-') else .06
            checks[case+'_position_bound_'+str(raw['lap_id'])]=metrics['position_error_max_m']<=max_error
            rows.append({'case':case,'lap':raw['lap_id'],'passed':raw['passed'],'completion_sim_s':metrics['completion_sim_s'],
                'completion_wall_s':metrics['completion_wall_s'],'real_time_factor':metrics['real_time_factor'],
                'position_error_max_cm':metrics['position_error_max_m']*100,'position_error_p95_cm':metrics['position_error_p95_m']*100,
                'post_clearance_min_cm':metrics['minimum_post_clearance_m']*100,
                'B_north_attempt':b['attempt'],'B_north_s':b['elapsed_s'],'counts':metrics['counts'],
                'plates':metrics['plates'],'road_violation_samples':metrics['road_violation_samples'],'gates':gates,
                'all_physical_and_image_checks':raw['checks']})
            if case=='sensor-stress':
                stress=raw['stress_statistics'][-1] if raw['stress_statistics'] else None
                stress_error=blackout_error_bound(raw,stress) if stress else {'passed':False}
    checks['all_person_views_two_actual_angles']=len(angles)==5*(a.main_laps+3) and all(v['passed'] for v in angles)
    checks['already_active_green_passes_without_new_red']=len(direct)>=2 and all(g['stop_wait_sim_s']<=2 for g in direct)
    checks['normal_signal_service_timeouts_zero']=bool(native) and all(d['timeouts']==d['failures']==d['requests']==0 for d in native)
    engines=[d['native_engine'] for d in native if d.get('native_engine')]
    checks['native_updates_apply_without_mismatch']=bool(engines) and max(e['updates'] for e in engines)>=12 and all(e['entities']==12 and e['poses_match'] and e['applied']==e['updates'] and e['mismatch_iterations']==0 and e['max_apply_delay_sim_ms']<=2 for e in engines)
    integrity=json.loads((F/'signal-integrity.json').read_text())
    checks['independent_actual_lens_observer']=integrity['passed']
    checks['sensor_noise_drops_and_blackout_recovery']=bool(stress) and stress['blackout_finished'] and stress['blackout_stop_passed'] and all(c['published']>100 and c['dropped']>0 and c['blackout']>0 for c in stress['counts'].values())
    checks['blackout_including_stale_position_error_bound']=stress_error['passed']
    hashes=json.loads((F/'runtime-hashes.json').read_text())
    checks['frozen_runtime_sources_and_binary']=all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h for p,h in hashes.items())
    checks['regression_counterexamples']=json.loads((F/'control-regression.json').read_text())['passed']
    checks['actual_callback_regression']=json.loads((F/'callback-regression.json').read_text())['passed']
    checks['original_world_geometry_preserved']=json.loads((F/'world-preservation.json').read_text())['passed']
    checks['gpu_ocr_and_plugin_preflight']=json.loads((F/'preflight.json').read_text())['passed']
    negative=json.loads((F/'image-negative-counterexample.json').read_text())
    checks['independent_image_ocr_rejects_wrong_expected_frame']=(not negative['passed'] and negative['events'][0]['rendered_frame_id']==3 and negative['events'][0]['expected_frame_id']==9999)
    recovery=json.loads((F/'suite-processes.json').read_text())
    checks['image_postprocessing_completed']=(recovery['passed'] and recovery['source_hashes_unchanged'] and
        (recovery.get('original_live_suite_exit',0)==0 or recovery.get('verification_wall_s',float('inf'))<90))
    result={'passed':all(checks.values()),'checks':checks,'laps':rows,'two_angle_evidence':angles,
        'already_active_green_passes':len(direct),'signal_integrity':{k:v for k,v in integrity.items() if k!='actual_model_pose_transitions'},
        'independently_observed_signal_changes':len(integrity['actual_model_pose_transitions']),
        'native_max_apply_delay_sim_ms':max(e['max_apply_delay_sim_ms'] for e in engines),
        'sensor_stress':stress,'blackout_error_bound':stress_error,'source_sha256':hashes,
        'postprocessing_recovery':recovery,
        'baseline_last_verified':{'completion_sim_s':181.7,'B_north_attempt':1,'B_north_s':9.1,'lower_stop_wait_s':24.1},
        'excluded_development_attempts':['attempt-01','attempt-02-phase-freshness','attempt-03-persistent-service-timeout','attempt-04-completion-status','attempt-05-image-verifier-timeout'],
        'limits':['Supplied fixed scene and known synchronized 10/15/3-second lamps','Phase inferred from camera edges; no simulator state subscribed by control','Cold unknown green without any observed phase anchor still cannot prove remaining time; late green below whole-body budget waits','Sensor perturbation strength and initial offsets explicitly bounded; not arbitrary scenes']}
    (F/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({'passed':result['passed'],'checks':checks,'laps':[{k:r[k] for k in ['case','lap','completion_sim_s','position_error_max_cm','B_north_attempt','B_north_s']} for r in rows]},ensure_ascii=False,indent=2))
    return 0 if result['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
