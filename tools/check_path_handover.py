"""Adversarial cached-path handover and visual heading calibration checks."""
import argparse,json,math,os,tempfile
from pathlib import Path
from types import SimpleNamespace as S
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as NavPath
from community_demo.patrol import Patrol,WAYPOINTS
from community_demo.continuous_route import plan_batch,rebase_batch
from concurrent.futures import Future
from community_demo.route_path import GridClearance
from community_demo.imu_heading import PaintHeadingCalibration,map_heading_at
checks={};c=PaintHeadingCalibration()
for t,delta in [(1.,.04),(1.2,.04)]:c.update(t,delta,.003)
checks['one_or_two_frames_cannot_calibrate']=c.offset==0
for t in[1.4,1.6,1.8,2.]:c.update(t,.04-c.offset,.003)
checks['consistent_paint_recovers_spawn_heading']=abs(c.offset-.04)<1e-9
checks['calibration_does_not_shift_imu_source_time']=map_heading_at([(3.,1.,.1)],3.06,c.offset)is None
checks['imu_rotation_preserved_after_alignment']=abs(map_heading_at([(3.,1.,.1)],3.04,c.offset)-1.044)<1e-9
before=c.offset;c.update(1.8,-.08,.003);checks['replayed_visual_frame_rejected']=c.offset==before
c.update(8.,-.08,.003);checks['calibration_window_freezes_later_ambiguous_paint']=c.offset==before
for kind,values in [('ambiguous',[(.04,0.)]*6),('outliers',[(.04,.003),(-.04,.003)]*3),('unbounded',[(.2,.1)]*6)]:
 c=PaintHeadingCalibration()
 for i,(delta,gain)in enumerate(values):c.update(1.+i*.2,delta,gain)
 checks[kind+'_cannot_change_heading']=c.offset==0.
now=Time(seconds=10.);grid=GridClearance(210,210,.02,(-2.1,-2.1),[0]*44100);reference=NavPath()
for x,y in[(-.74,1.7),(-1.7,1.7),(-1.7,1.04)]:
 p=PoseStamped();p.pose.position.x=x;p.pose.position.y=y;reference.poses.append(p)
commands=[];evidence=[];warnings=[]
node=S(index=3,lap_id=1,run_id='test',pose=(-.72,1.7,math.pi),pose_source_ns=9_980_000_000,costmap=grid,costmap_stamp_ns=9_900_000_000,static_posts=[],get_clock=lambda:S(now=lambda:now),front_blocked=lambda:False,path_index=-1,path=None,path_attempts=0,get_logger=lambda:S(info=lambda s:None),path_pub=S(publish=lambda p:evidence.append(p)))
def fail(reason):node.path=None;node.path_index=-1;warnings.append(reason)
node.fail_path_request=fail;node.install_path=lambda p,b,e,prepared=None:Patrol.install_path(node,p,b,e,prepared)
prepared=Future();prepared.set_result(plan_batch([(-.74,1.7),(-1.7,1.7),(-1.7,1.04)],(-.74,1.7,math.pi),WAYPOINTS,3,4,grid,[],{}))
node.prefetched_path=((1,3),4,reference,9_500_000_000,prepared)
old=Path.cwd()
with tempfile.TemporaryDirectory()as tmp:
 try:
  os.chdir(tmp)
  checks['fresh_cached_reference_installed']=Patrol.take_prefetched_path(node,now)
  checks['cache_replanned_from_actual_pose']=bool(evidence)and math.dist((evidence[-1].poses[0].pose.position.x,evidence[-1].poses[0].pose.position.y),node.pose[:2])<1e-9
  for name,key,stamp,blocked in [('old_lap',(0,3),9_500_000_000,False),('old_route',(1,2),9_500_000_000,False),('stale',(1,3),4_000_000_000,False),('future',(1,3),11_000_000_000,False),('obstacle',(1,3),9_500_000_000,True)]:
   node.front_blocked=lambda:blocked;node.prefetched_path=(key,4,reference,stamp,prepared);n=len(evidence)
   checks[name+'_cached_reference_rejected']=not Patrol.take_prefetched_path(node,now)and len(evidence)==n
  node.front_blocked=lambda:False;node.prefetched_path=((1,3),4,reference,9_500_000_000,prepared);node.costmap_stamp_ns=7_000_000_000
  checks['fresh_reference_stale_grid_rejected']=not Patrol.take_prefetched_path(node,now)
  node.costmap_stamp_ns=9_900_000_000;node.costmap=GridClearance(210,210,.02,(-2.1,-2.1),[100]*44100);node.prefetched_path=((1,3),4,reference,9_500_000_000,prepared)
  checks['new_occupied_grid_rejects_previously_safe_reference']=not Patrol.take_prefetched_path(node,now)
 finally:os.chdir(old)
# Validate the fast connection independently, including original via/goal poses.
curve,metadata=prepared.result()
joined,meta=rebase_batch(curve,metadata,(-.72,1.705,math.pi+.01),grid,[])
checks['fast_handover_exact_measured_start']=joined is not None and math.dist(joined[0][:2],(-.72,1.705))<1e-9 and abs(math.atan2(math.sin(joined[0][2]-(math.pi+.01)),math.cos(joined[0][2]-(math.pi+.01))))<1e-9
checks['fast_handover_preserves_all_mandatory_vias']=joined is not None and all(math.dist(joined[segment['curve_end_index']][:2],WAYPOINTS[segment['waypoint_index']]['position'])<1e-8 for segment in meta['segments'])
checks['far_displaced_cached_start_rejected']=rebase_batch(curve,metadata,(-.64,1.7,math.pi),grid,[])[0]is None
checks['wrong_heading_cached_start_rejected']=rebase_batch(curve,metadata,(-.74,1.7,0.),grid,[])[0]is None
blocked=GridClearance(210,210,.02,(-2.1,-2.1),[-1]*44100)
checks['fast_handover_rechecks_unknown_grid']=rebase_batch(curve,metadata,(-.72,1.7,math.pi),blocked,[])[0]is None
pending=Future();cached=node.prefetched_path
node.prefetched_path=('old-valid',);node.prefetch_candidate=('new-pending',pending)
Patrol.promote_prepared_path(node)
checks['unfinished_refresh_does_not_evict_ready_path']=node.prefetched_path==('old-valid',)
pending.set_result((curve,metadata));Patrol.promote_prepared_path(node)
checks['completed_refresh_replaces_ready_path']=node.prefetched_path[0]=='new-pending'
report={'passed':all(checks.values()),'checks':checks,'scope':'Real cached-path install with changed measured start, live occupied/stale grids, wrong lap and source-time invalid data. No sensor truth fed to heading calibration.'}
p=argparse.ArgumentParser();p.add_argument('--output',required=True);a=p.parse_args();out=Path(a.output);out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2));raise SystemExit(0 if report['passed']else 1)
