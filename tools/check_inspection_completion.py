#!/usr/bin/env python3
"""Real patrol callbacks: incomplete observations cannot advance or complete."""
import argparse,copy,json,math,os,tempfile
from pathlib import Path
from types import SimpleNamespace as S
import numpy as np
from rclpy.time import Time
from community_demo.patrol import Patrol,ROUTE,OBSERVATIONS
from community_demo.inspection_policy import inspection_complete,recognition_complete

class Pub:
    def __init__(self):self.messages=[]
    def publish(self,m):self.messages.append(m.data)

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True);args=parser.parse_args()
    required=[v for views in OBSERVATIONS.values() for v,a in views]
    good=[{'run_id':'unit','lap_id':1,'view':v,'confirmed':True} for v in required]
    checks={'all_required_views_accepted':inspection_complete(good,required,'unit',1),
            'missing_view_rejected':not inspection_complete(good[:-1],required,'unit',1),
            'duplicate_view_rejected':not inspection_complete(good[:-1]+[good[0]],required,'unit',1),
            'other_lap_rejected':not inspection_complete(good,required,'unit',2)}
    bad=copy.deepcopy(good);bad[0]['confirmed']=False
    checks['unconfirmed_view_rejected']=not inspection_complete(bad,required,'unit',1)
    bad[0]['confirmed']='true'
    checks['nonboolean_confirmation_rejected']=not inspection_complete(bad,required,'unit',1)
    totals={'residents':8,'visitors':2,'total':10};plates=['plate-A','plate-B','plate-C']
    checks.update(camera_totals_complete=recognition_complete(totals,plates,10,3),
                  one_unseen_person_rejected=not recognition_complete({'residents':7,'visitors':2,'total':9},plates,10,3),
                  missing_plate_rejected=not recognition_complete(totals,plates[:2],10,3),
                  duplicate_plate_rejected=not recognition_complete(totals,['plate-A']*3,10,3),
                  inconsistent_person_counts_rejected=not recognition_complete({'residents':7,'visitors':2,'total':10},plates,10,3))
    now=Time(seconds=16);warnings=[];statuses=[];laps=[];commands=[]
    p=S(observation_start=Time(seconds=0),observation_align=False,pose=(0.,0.,math.pi),observation_index=0,
        observation_feedback={},feedback_update=now,observation_attempt=0,observation_sample=0,sample_start=now,
        pan_pub=Pub(),observation_pub=Pub(),inspection_pub=Pub(),run_id='unit',lap_id=1,
        camera_frame=np.zeros((32,32,3),np.uint8),saved_views=[],observation_outcomes=[],fault=None,
        publish_command=lambda *a:commands.append(a),log_status=statuses.append,get_logger=lambda:S(warning=warnings.append))
    previous=Path.cwd()
    with tempfile.TemporaryDirectory(prefix='aic-inspection-check-') as tmp:
        try:
            os.chdir(tmp);advanced=Patrol.observe(p,'observe_A_north',now)
        finally:os.chdir(previous)
    checks.update(timeout_does_not_advance=not advanced and p.observation_index==0,
                  timeout_latches_fault=bool(p.fault) and p.fault.startswith('FAULT inspection view'),
                  timeout_retains_failed_evidence=len(p.observation_outcomes)==1 and p.observation_outcomes[0]['confirmed'] is False)
    p.completed=False;p.tilt=0.;p.pose=(1.1,1.7,0.);p.pose_update=now;p.pose_source_ns=now.nanoseconds
    p.require_vision=True;p.perception_update=now;p.perception_stamp=16.;p.scan_update=now;p.scan_source_ns=now.nanoseconds
    p.quality_update=now;p.localization_quality={'state':'tracking'};p.index=len(ROUTE)-1;p.distance_travelled=12.;p.motion_probe=None
    p.recognition_counts=totals;p.recognition_plates=plates;p.expected_people=10;p.expected_plates=3
    p.get_clock=lambda:S(now=lambda:now);p.update_signal_context=lambda:None;p.check_path_deadline=lambda t:None;p.publish_lap=laps.append
    Patrol.tick(p);checks['fault_keeps_chassis_stopped']=commands[-1]==() and not p.completed and not laps
    p.fault=None;p.observation_outcomes=good[:-1];Patrol.tick(p)
    checks['final_route_without_all_views_cannot_complete']=not p.completed and not laps and p.fault.startswith('FAULT incomplete inspection')
    p.fault=None;p.index=len(ROUTE)-1;p.observation_outcomes=good;p.recognition_plates=plates[:2];Patrol.tick(p)
    checks['all_views_but_partial_totals_cannot_complete']=not p.completed and not laps and p.fault.startswith('FAULT incomplete inspection')
    p.fault=None;p.index=len(ROUTE)-1;p.recognition_plates=plates;Patrol.tick(p)
    checks['complete_evidence_still_finishes_route']=p.completed and laps==['completed']
    result={'passed':all(checks.values()),'checks':checks,'scope':'Synthetic inputs to production inspection and final-route callbacks; actual successful perception remains a live-lap check.'}
    out=Path(args.output);out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result));return 0 if result['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
