#!/usr/bin/env python3
"""Independent image-time propagation, freshness and physical-turn counterexamples."""
import argparse,json,math
from collections import deque
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from rclpy.time import Time
from sensor_msgs.msg import Image
from community_demo.pose_sync import interpolate,field_pose
from community_demo.localization import Localization
from community_demo.patrol import ROUTE
from community_demo.road_rules import safe_pose
p=argparse.ArgumentParser();p.add_argument('--output',required=True);a=p.parse_args();checks={}
# Wheel robot moves 1 m/s. Image at .025 must be registered at .025,
# then a .02 m image correction must yield current .12 at t=.1.
h=deque([(0.,(0.,0.,0.)),(.05,(.05,0.,0.)),(.1,(.1,0.,0.))])
w=interpolate(h,.025);checks['interpolates_at_image_stamp']=w==(.025,0.,0.)
back=field_pose((.1,0.,0.),h[-1][1],w)
now=field_pose((back[0]+.02,back[1],back[2]),w,h[-1][1])
checks['image_correction_preserves_travel_since_exposure']=abs(now[0]-.12)<1e-10
checks['does_not_extrapolate_future_or_before_history']=interpolate(h,.2) is None and interpolate(h,-.1) is None
checks['long_gap_not_filled_with_invented_motion']=interpolate([(0.,(0.,0.,0.)),(2.,(1.,0.,0.))],1.) is None
turn=interpolate([(0.,(0.,0.,math.pi-.01)),(.1,(0.,0.,-math.pi+.01))],.05)
checks['yaw_interpolation_wraps_short_arc']=abs(abs(turn[2])-math.pi)<1e-10
clock=SimpleNamespace(now=lambda:Time(seconds=10))
node=SimpleNamespace(get_clock=lambda:clock,camera_received=5.,camera_source_time=5.,pending_image=None)
blank=Image();blank.encoding='bgr8';blank.height=96;blank.width=128;blank.step=384;blank.data=np.zeros((96,128,3),np.uint8).tobytes();blank.header.stamp=Time(seconds=8).to_msg()
Localization.queue_image(node,blank);checks['stale_image_does_not_refresh_camera']=node.camera_received==5. and node.pending_image is None
blank.header.stamp=Time(seconds=10).to_msg();Localization.queue_image(node,blank)
checks['fresh_image_queue_keeps_source_stamp']=node.camera_source_time==10. and node.pending_image is blank
local=SimpleNamespace(get_clock=lambda:clock,frame_time=0.,wheel_samples=deque([(9.95,(0.,0.,0.)),(10.05,(.01,0.,0.))]),previous=np.array([.01,0.,0.]),xy=np.array([1.1,1.7]),yaw=math.pi,pan_samples=deque([(10.,0.,0.)]),last_match=5.,image_odom_delta=None)
Localization.image(local,blank);checks['blank_pixels_do_not_refresh_registration']=local.last_match==5. and local.registration_points==0
way=next(x for x in ROUTE if x[0]=='lower_south')
checks['bottom_turn_full_body_stays_in_corridor']=all(safe_pose(way[1],way[2],-math.pi/2+i*math.pi/2/100) for i in range(101))
# Floating subtraction wrongly rejects 24.1-23.95 as greater than .15.
# Exercise the actual publisher at exactly the threshold and one ns beyond it.
from nav_msgs.msg import Odometry
clock_ns=[24_100_000_000];published=[]
state=SimpleNamespace(get_clock=lambda:SimpleNamespace(now=lambda:Time(nanoseconds=clock_ns[0])),pending_image=None,wheel_samples=[],odom_time=Time(nanoseconds=23_950_000_000).to_msg(),odom_received=24.1,camera_received=24.1,camera_source_time=24.1,unmatched_distance=0.,encoder_age=0.,image_odom_delta=0.,registration_stamp=24.,registration_corrections=0,axis_improvements=None,score=.01,registration_points=100,last_match=24.,xy=np.array([1.1,1.7]),yaw=math.pi,quality=SimpleNamespace(publish=lambda m:None),status=SimpleNamespace(publish=lambda m:None),tf=SimpleNamespace(sendTransform=lambda m:None),poses=SimpleNamespace(publish=published.append))
Localization.publish(state);checks['exact_150ms_source_age_is_accepted']=len(published)==1 and published[0].header.stamp==state.odom_time
clock_ns[0]+=1;Localization.publish(state);checks['older_than_150ms_is_still_withheld']=len(published)==1
# Exercise production callbacks with corrupt payloads and stale evidence.
import copy
from types import MethodType
from sensor_msgs.msg import JointState
from community_demo.perception import Perception
from community_demo.patrol import Patrol
from community_demo.signal_gate import VisualPhaseClock,FreshGreenGate
bad=copy.deepcopy(blank);bad.data=bad.data[:-1]
old_pending=node.pending_image;old_camera=node.camera_received
Localization.queue_image(node,bad)
checks['truncated_image_does_not_refresh_localization']=node.pending_image is old_pending and node.camera_received==old_camera
vision=SimpleNamespace(get_clock=lambda:clock,last_stamp=None,frame=None)
Perception.on_camera(vision,bad)
checks['truncated_image_cannot_crash_or_refresh_perception']=vision.frame is None and vision.last_stamp is None
bad=copy.deepcopy(blank);bad.step=1
Perception.on_camera(vision,bad)
checks['invalid_stride_is_rejected']=vision.frame is None
future=copy.deepcopy(blank);future.header.stamp=Time(seconds=11).to_msg()
Localization.queue_image(node,future);Perception.on_camera(vision,future)
checks['future_image_cannot_refresh_evidence']=node.camera_received==old_camera and vision.frame is None
odom=Odometry();odom.header.stamp=Time(seconds=10).to_msg();odom.pose.pose.orientation.w=1.;odom.pose.pose.position.x=float('nan')
loc=SimpleNamespace(get_clock=lambda:clock,wheel_samples=deque(),odom_received=5.,previous=None)
Localization.odom(loc,odom)
checks['nan_odometry_does_not_refresh_or_poison_pose']=loc.odom_received==5. and loc.previous is None and not loc.wheel_samples
odom.pose.pose.position.x=0.;odom.pose.pose.orientation.w=0.
Localization.odom(loc,odom);checks['zero_quaternion_does_not_create_navigation_pose']=loc.previous is None
enc=JointState();enc.header.stamp=Time(seconds=10).to_msg();enc.name=['camera_pan_joint'];enc.position=[float('nan')];enc.velocity=[0.]
aligned=SimpleNamespace(pan_samples=deque())
Localization.on_joint_state(aligned,enc);Perception.on_joint(aligned,enc)
checks['nonfinite_encoder_never_counts_as_aligned']=not aligned.pan_samples
enc.position=[0.];enc.velocity=[];Perception.on_joint(aligned,enc)
checks['missing_encoder_velocity_is_not_stable']=not aligned.pan_samples
clock_n=[11_000_000_000]
phase=VisualPhaseClock();control=SimpleNamespace(get_clock=lambda:SimpleNamespace(now=lambda:Time(nanoseconds=clock_n[0])),light_source_ns=None,light_state='unknown',light_update=None,phase_clock=phase,signal_gate=FreshGreenGate(phase),index=0,completed=False,signal_context=None,crossing_gate=None)
control.update_signal_context=MethodType(Patrol.update_signal_context,control)
from std_msgs.msg import String
def lamp(source,colour):Patrol.on_stamped_light(control,String(data=json.dumps({'source_stamp_ns':source,'colour':colour})))
lamp(10_000_000_000,'green');checks['stale_visual_light_does_not_refresh_release']=control.light_update is None
lamp(11_001_000_000,'green');checks['future_visual_light_cannot_invent_an_anchor']=control.light_update is None and phase.anchor is None
lamp(10_950_000_000,'red');clock_n[0]=11_100_000_000;lamp(11_050_000_000,'green')
anchor=phase.anchor;streak=control.signal_gate.green_streak
lamp(11_050_000_000,'green');lamp(11_040_000_000,'red')
checks['replayed_visual_frame_cannot_extend_green_streak']=control.signal_gate.green_streak==streak and control.light_state=='green'
checks['visual_edge_uses_exposure_not_delivery_time']=anchor is not None and anchor[1]==11.05 and phase.remaining_lower(11.1)<=14.65
invalid=VisualPhaseClock();invalid.observe('yellow',10.);invalid.observe('green',10.1)
checks['impossible_yellow_to_green_cannot_invent_cycle']=invalid.anchor is None
from community_demo.road_rules import corridor_recovery,predict,safe_pose
biased=(-.099,-1.845,.305)
checks['noise_biased_corner_reproduces_unsafe_turn']=not all(safe_pose(*predict(*biased,0.,.5,t)) for t in (.1,.3,.5))
step=corridor_recovery(biased,(1.1,-1.825),[])
checks['constrained_recovery_repositions_without_crossing_paint']=step is not None and all(safe_pose(*predict(*biased,*step,t)) for t in (.1,.3,.5))
checks['recovery_never_drives_away_from_goal']=corridor_recovery(biased,(-1.,-1.8),[]) is None
checks['recovery_never_bypasses_a_physical_post']=corridor_recovery(biased,(1.1,-1.825),[{'x':-.055,'y':-1.825,'yaw':0.,'width':.05,'depth':.05}]) is None
from audit_patrol_cycles import align_signal_evidence
history={665_800_000_000:'green',675_000_000_000:'red'}
truth=[{'t':665.8,'phase':'red'},{'t':666.,'phase':'red'},{'t':675.,'phase':'green'}]
ages=align_signal_evidence(truth,history)
checks['auditor_pairs_backlogged_truth_with_source_time_lenses']=truth[0]['phase']=='green' and truth[2]['phase']=='red'
checks['missing_lens_sample_is_unknown_not_invented_phase']=truth[1]['phase']=='unknown' and ages[1]>.075
from audit_patrol_cycles import pose_at_source
samples={1_000_000_000:(0.,0.,0.),1_100_000_000:(.1,0.,0.),2_000_000_000:(.9,0.,0.)};times=sorted(samples)
checks['future_estimate_cannot_be_paired_with_old_truth']=pose_at_source(samples,times,1_050_000_000)==(.05,0.,0.)
checks['pose_metric_never_extrapolates_or_bridges_blackout']=pose_at_source(samples,times,1_500_000_000) is None and pose_at_source(samples,times,500_000_000) is None
checks['isolated_exact_source_sample_still_is_measured']=pose_at_source(samples,times,2_000_000_000)==(.9,0.,0.)
from community_demo.patrol import OBSERVATION_HEADING
from tempfile import TemporaryDirectory
import os
def plate_observation(ready=True,elapsed=1.9,feedback_age=.1,view='plate_1'):
    now=Time(seconds=20.)
    sink=SimpleNamespace(publish=lambda m:None)
    state=SimpleNamespace(observation_align=False,observation_start=Time(seconds=20.-elapsed),
        observation_index=0,observation_attempt=0,observation_sample=0,sample_start=None,
        pose=(1.1,-1.7,OBSERVATION_HEADING['observe_plate_1']),
        observation_feedback={'view':view,'attempt':0,'ready':ready},
        feedback_update=Time(seconds=20.-feedback_age),pan_pub=sink,observation_pub=sink,
        inspection_pub=sink,publish_command=lambda *args:None,log_status=lambda s:None,
        camera_frame=np.zeros((48,64,3),np.uint8),run_id='regression',lap_id=1,
        observation_outcomes=[],saved_views=[])
    previous=os.getcwd()
    with TemporaryDirectory() as folder:
        try:
            os.chdir(folder)
            return Patrol.observe(state,'observe_plate_1',now)
        finally:os.chdir(previous)
checks['confirmed_plate_does_not_wait_for_redundant_timer']=plate_observation()
checks['unconfirmed_plate_cannot_finish_early']=not plate_observation(ready=False)
checks['plate_still_requires_minimum_stable_observation']=not plate_observation(elapsed=1.7)
checks['stale_plate_feedback_cannot_finish_early']=not plate_observation(feedback_age=.8)
checks['different_view_plate_feedback_cannot_finish_early']=not plate_observation(view='plate_2')
def quality_hold(reason=None):
    now=Time(seconds=20.);pan=[];commands=[]
    state=SimpleNamespace(get_clock=lambda:SimpleNamespace(now=lambda:now),update_signal_context=lambda:None,
        check_path_deadline=lambda now:None,tilt=0.,fault=None,completed=False,pose=(1.1,1.7,math.pi),pose_update=Time(seconds=19.95),
        pose_source_ns=19_950_000_000,require_vision=True,perception_update=Time(seconds=19.95),
        perception_stamp=19.95,scan_update=Time(seconds=19.95),scan_source_ns=19_950_000_000,quality_update=Time(seconds=19.95),
        localization_quality={'state':'lost','reason':reason},motion_probe=None,
        publish_command=lambda *args:commands.append(args),pan_pub=SimpleNamespace(publish=pan.append),
        log_status=lambda s:None)
    Patrol.tick(state)
    return commands,pan
commands,pan=quality_hold('source or receipt freshness')
checks['freshness_only_quality_loss_stops_without_resetting_pan']=commands==[()] and not pan
commands,pan=quality_hold()
checks['actual_registration_loss_still_stops_and_recenters']=commands==[()] and len(pan)==1 and pan[0].data==0.
report={'passed':all(checks.values()),'checks':checks,'scope':'Actual interpolation, propagation, queue freshness and full-body turn geometry; live checks separate'}
out=Path(a.output);out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report));raise SystemExit(0 if report['passed'] else 1)
