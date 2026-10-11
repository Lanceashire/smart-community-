#!/usr/bin/env python3
"""Real patrol callbacks: incomplete observations cannot advance or complete."""
import argparse,copy,json,math,os,tempfile
from pathlib import Path
from types import SimpleNamespace as S
import numpy as np
from rclpy.time import Time
from community_demo.patrol import Patrol,ROUTE,OBSERVATIONS
from community_demo.task_geometry import FINISH
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
    p.completed=False;p.tilt=0.;p.pose=FINISH;p.pose_update=now;p.pose_source_ns=now.nanoseconds
    p.require_vision=True;p.perception_update=now;p.perception_stamp=16.;p.scan_update=now;p.scan_source_ns=now.nanoseconds
    p.quality_update=now;p.localization_quality={'state':'tracking'};p.index=len(ROUTE)-1;p.path_index=-1;p.distance_travelled=12.;p.motion_probe=None
    p.recognition_counts=totals;p.recognition_plates=plates;p.expected_people=10;p.expected_plates=3
    p.get_clock=lambda:S(now=lambda:now);p.update_signal_context=lambda:None;p.check_path_deadline=lambda t:None;p.publish_lap=laps.append
    Patrol.tick(p);checks['fault_keeps_chassis_stopped']=commands[-1]==() and not p.completed and not laps
    p.fault=None;p.observation_outcomes=good[:-1];Patrol.tick(p)
    checks['final_route_without_all_views_cannot_complete']=not p.completed and not laps and p.fault.startswith('FAULT incomplete inspection')
    p.fault=None;p.index=len(ROUTE)-1;p.observation_outcomes=good;p.recognition_plates=plates[:2];Patrol.tick(p)
    checks['all_views_but_partial_totals_cannot_complete']=not p.completed and not laps and p.fault.startswith('FAULT incomplete inspection')
    p.fault=None;p.index=len(ROUTE)-1;p.recognition_plates=plates;Patrol.tick(p)
    checks['complete_evidence_still_finishes_route']=p.completed and laps==['completed']
    # Exercise the same terminal branch that prevented the 8mm trial from
    # finishing; the steering extension must never authorize endless travel.
    p.completed=False;p.fault=None;p.index=len(ROUTE)-1;p.path_index=p.index;p.path_curved=True;p.path_goal_yaw=FINISH[2]
    p.pose=(FINISH[0]+.04,FINISH[1],FINISH[2]);p.path=object();before=len(laps)
    Patrol.tick(p)
    checks['terminal_overshoot_stops_and_replans']=commands[-1]==() and p.path_index==-1 and p.path is None and p.index==len(ROUTE)-1 and len(laps)==before
    for _ in range(2):
        p.path_index=p.index;p.path=object();Patrol.tick(p)
    checks['terminal_replans_are_bounded']=p.arrival_replans==3 and p.fault.startswith('FAULT terminal goal missed') and commands[-1]==()
    p.fault=None;p.path_index=p.index;p.lap_id=2
    Patrol.tick(p)
    checks['new_lap_does_not_inherit_arrival_retry_count']=p.arrival_replans==1 and p.fault is None and p.path_index==-1
    p.fault=None;p.index=next(i for i,v in enumerate(ROUTE)if v[0]=='bottom_east');p.path_index=p.index
    p.pose=(-.1,-1.67,-math.pi/2);p.path_goal_yaw=FINISH[2]/2;p.path_curved=True
    p.front_blocked=lambda:True;p.blocked_since=None;p.path_heading_error=lambda:None;p.scan=S(ranges=[.1])
    count=p.arrival_replans;Patrol.tick(p)
    checks['curve_entry_above_terminal_plane_is_not_overshoot']=p.arrival_replans==count and p.path_index==p.index and p.fault is None and statuses[-1].startswith('BLOCKED')
    p.index=len(ROUTE)-1;p.path_goal_yaw=FINISH[2]
    p.fault=None;p.path_index=p.index;p.path_curved=False;p.pose=(*FINISH[:2],-math.pi/2)
    p.costmap_stamp_ns=now.nanoseconds-3_000_000_000
    Patrol.tick(p)
    checks['stale_grid_blocks_terminal_pivot']=commands[-1]==() and p.path_index==-1 and statuses[-1].startswith('ARRIVAL_HOLD')
    from community_demo.route_path import GridClearance
    p.path_index=p.index;p.costmap_stamp_ns=now.nanoseconds;p.costmap=GridClearance(210,210,.02,(-2.1,-2.1),[0]*44100);p.static_posts=[]
    p.heading_pid=S(calculate=lambda e:e);p.command_guard=None
    Patrol.tick(p)
    checks['checked_terminal_heading_aligns_without_advancing']=commands[-1][0]==0. and commands[-1][1]>0 and p.index==len(ROUTE)-1 and len(laps)==before
    # A faster patrol tick must not exhaust retries against a single grid
    # published before the obstacle cleared. New evidence enables a retry;
    # a frozen publisher still faults within a finite interval.
    retry=S(path_retry_deadline=15.,path_retry_grid_ns=10_000_000_000,
            costmap_stamp_ns=10_000_000_000,fault=None,path_attempts=1)
    checks['same_grid_cannot_spend_more_planning_attempts']=not Patrol.path_retry_ready(retry,Time(seconds=11.)) and retry.path_attempts==1
    retry.costmap_stamp_ns=10_500_000_000
    checks['new_grid_enables_bounded_planning_retry']=Patrol.path_retry_ready(retry,Time(seconds=11.)) and retry.path_retry_deadline is None
    retry.path_retry_deadline=15.;retry.path_retry_grid_ns=retry.costmap_stamp_ns
    checks['missing_grid_update_faults_at_retry_deadline']=not Patrol.path_retry_ready(retry,Time(seconds=15.)) and retry.fault.startswith('FAULT no updated live grid')
    result={'passed':all(checks.values()),'checks':checks,'scope':'Synthetic inputs to production inspection and final-route callbacks; actual successful perception remains a live-lap check.'}
    out=Path(args.output);out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result));return 0 if result['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
