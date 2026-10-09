#!/usr/bin/env python3
"""Regression of actual production callbacks; captured outputs never reach actuators."""
import argparse,copy,json,math,struct
from pathlib import Path
from types import SimpleNamespace,MethodType
from rclpy.time import Time
from sensor_msgs.msg import LaserScan,PointCloud2,PointField
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as NavPath
from action_msgs.msg import GoalStatus
from community_demo.patrol import Patrol
from community_demo.localization import Localization
from community_demo.signal_gate import FreshGreenGate,VisualPhaseClock
from community_demo.sensor_validation import decode_xyz_cloud

clock=[21.7];sink=SimpleNamespace(publish=lambda m:None);checks={}
def node_at(index=1):
    commands,statuses,decisions=[],[],[]
    phase=VisualPhaseClock();phase.observe('red',9.9);phase.observe('green',10.)
    gate=FreshGreenGate(phase)
    for t in [20.7,20.8,20.9,21.,21.1]:phase.observe('green',t);gate.observe('green',t)
    n=SimpleNamespace(get_clock=lambda:SimpleNamespace(now=lambda:Time(seconds=clock[0])),
        get_logger=lambda:SimpleNamespace(warning=lambda m:None),
        index=index,completed=False,signal_context='top' if index==1 else None,
        crossing_gate=None,signal_gate=gate,phase_clock=phase,tilt=0.,fault=None,
        run_id='boundary-regression',lap_id=1,observation_align=False,
        pose=(.52,1.7,math.pi),pose_update=Time(seconds=clock[0]-.05),pose_source_ns=round((clock[0]-.05)*1e9),
        require_vision=True,perception_update=Time(seconds=clock[0]-.05),perception_stamp=clock[0]-.05,
        scan_update=Time(seconds=clock[0]-.05),scan_source_ns=round((clock[0]-.05)*1e9),
        quality_update=Time(seconds=clock[0]-.05),localization_quality={'state':'tracking'},motion_probe=None,
        light_update=Time(seconds=21.1),light_source_ns=21_100_000_000,light_state='green',
        waiting_for_green=False,green_streak=0,pan_pub=sink,
        signal_decisions=SimpleNamespace(publish=lambda m:decisions.append(json.loads(m.data))),
        log_status=statuses.append,command_pub=SimpleNamespace(publish=commands.append),
        static_posts=[],post_replan_time=-1e9,road_recovery_started=None,road_recovery_origin=None,
        blocked_since=None,clear_scans=0,recovery_count=0,front_blocked=lambda:False,
        path_index=index,path_pending=False,path=SimpleNamespace(poses=[1]),path_request_token=0,
        path_request_deadline=None,path_goal_handle=None,path_retry_index=None,path_attempts=0,
        path_pub=sink,path_heading_error=lambda:0.,path_points=[(.52,1.7),(-.74,1.7)],path_nearest=0,
        speed_pid=SimpleNamespace(calculate=lambda e:.24),heading_pid=SimpleNamespace(calculate=lambda e:0.),
        publish_lap=lambda phase:None,distance_travelled=12.)
    for method in ['update_signal_context','fresh_signal','publish_command','check_path_deadline',
                   'cancel_path_request','fail_path_request','request_path']:
        setattr(n,method,MethodType(getattr(Patrol,method),n))
    return n,commands,statuses,decisions

n,cmd,status,dec=node_at();Patrol.tick(n)
checks['old_green_cannot_advance_stop']=n.index==1 and not dec
checks['remaining_budget_uses_current_clock']=not n.signal_gate.ready_at(21.7)
clock[0]=20.8;n,cmd,status,dec=node_at()
n.light_update=Time(seconds=20.8);n.light_source_ns=20_800_000_000
# Five distinct camera observations before a current, sufficiently long green.
n.signal_gate.reset()
for t in [20.4,20.5,20.6,20.7,20.8]:n.signal_gate.observe('green',t)
Patrol.tick(n);checks['fresh_sufficient_green_releases']=n.index==2 and len(dec)==1
clock[0]=21.7;n.light_update=Time(seconds=21.7);n.light_source_ns=21_700_000_000
n.crossing_gate.observe('green',21.7);n.publish_command(.24,0.)
checks['late_budget_after_release_blocks_before_line']=cmd[-1].twist.linear.x==0.
original_pose=n.pose;n.pose=(-.1,-.37,-math.pi/2);n.publish_command(.24,0.)
checks['lower_gate_protection_reaches_actual_front_line']=cmd[-1].twist.linear.x==0. and n.crossing_gate is not None
n.pose=original_pose
clock[0]=49.
for t in [48.6,48.7,48.8,48.9,49.]:n.crossing_gate.observe('green',t)
n.phase_clock.observe('green',49.);n.light_update=Time(seconds=49.);n.light_source_ns=49_000_000_000
n.publish_command(.24,0.)
checks['next_camera_confirmed_green_can_recover']=cmd[-1].twist.linear.x==.24
n.light_source_ns=48_000_000_000;n.publish_command(.24,0.)
checks['fresh_receipt_does_not_hide_old_light_source']=cmd[-1].twist.linear.x==0.

clock[0]=10.
s=SimpleNamespace(get_clock=lambda:SimpleNamespace(now=lambda:Time(seconds=clock[0])),
                  scan=None,scan_update=None,scan_source_ns=None,clear_scans=0)
s.front_blocked=MethodType(Patrol.front_blocked,s)
scan=LaserScan();scan.angle_min=-math.pi;scan.angle_increment=math.pi/360
scan.angle_max=scan.angle_min+719*scan.angle_increment;scan.range_min=.17;scan.range_max=6.
scan.header.stamp=Time(seconds=1.).to_msg();scan.ranges=[math.inf]*720
for _ in range(5):Patrol.on_scan(s,scan)
checks['stale_scan_never_refreshes_or_clears']=s.scan_update is None and s.clear_scans==0
scan.header.stamp=Time(seconds=10.).to_msg();scan.ranges=[math.nan]*720
for _ in range(5):Patrol.on_scan(s,scan)
checks['nan_scan_never_refreshes_or_clears']=s.scan_update is None and s.clear_scans==0
scan.ranges=[math.inf]*720
Patrol.on_scan(s,scan)
for _ in range(5):Patrol.on_scan(s,scan)
checks['same_scan_counted_once']=s.clear_scans==1
for t in [10.1,10.2,10.3,10.4]:
    clock[0]=t;scan.header.stamp=Time(seconds=t).to_msg();Patrol.on_scan(s,scan)
checks['five_distinct_valid_clear_scans_release']=s.clear_scans==5
scan.header.stamp=Time(seconds=10.5).to_msg();clock[0]=10.5;scan.ranges[360]=.2
Patrol.on_scan(s,scan);checks['valid_near_obstacle_resets_confirmation']=s.clear_scans==0 and s.front_blocked()
clock[0]=11.2;n,cmd,status,dec=node_at(16);n.pose=(1.1,0.,math.pi/2)
n.scan_source_ns=10_500_000_000;n.scan_update=Time(seconds=11.2)
Patrol.tick(n);checks['old_scan_source_stops_even_with_fresh_receipt']=status[-1]=='WAIT_FRESH_SCAN' and cmd[-1].twist.linear.x==0.

clock[0]=10.
m=PointCloud2();m.header.stamp=Time(seconds=10.).to_msg();m.width=m.height=1;m.point_step=m.row_step=12
m.fields=[PointField(name=c,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,c in enumerate('xyz')]
published=[];local=SimpleNamespace(scans=SimpleNamespace(publish=published.append),cloud_source_ns=None,get_clock=s.get_clock)
m.data=b'\0'*11
Localization.cloud(local,m);checks['truncated_cloud_dropped_without_exception']=not published and local.cloud_source_ns is None
m.data=struct.pack('<fff',1.,0.,0.);Localization.cloud(local,m)
checks['valid_cloud_publishes_after_bad_frame']=len(published)==1 and math.isinf(published[0].ranges[0])
Localization.cloud(local,m);checks['replayed_cloud_not_republished']=len(published)==1
m.is_bigendian=True;m.height=2;m.row_step=16;m.data=struct.pack('>fff',1.,2.,3.)+b'pad!'+struct.pack('>fff',4.,5.,6.)+b'pad!'
xyz=decode_xyz_cloud(m);checks['big_endian_padded_rows_decoded']=xyz is not None and xyz.tolist()==[[1.,2.,3.],[4.,5.,6.]]
m.fields[0].datatype=PointField.FLOAT64;checks['unsupported_xyz_type_dropped']=decode_xyz_cloud(m) is None

class Future:
    def __init__(self,result=None,error=None):self.value=result;self.error=error;self.callback=None
    def result(self):
        if self.error:raise self.error
        return self.value
    def add_done_callback(self,cb):self.callback=cb
    def fire(self):self.callback(self)
class Handle:
    accepted=True
    def __init__(self):self.cancelled=0;self.future=Future()
    def cancel_goal_async(self):self.cancelled+=1
    def get_result_async(self):return self.future
class Planner:
    def __init__(self):self.futures=[]
    def server_is_ready(self):return True
    def send_goal_async(self,goal):
        f=Future(Handle());self.futures.append(f);return f

clock[0]=100.;n,cmd,status,dec=node_at(2);n.planner=Planner()
n.request_path(-.74,1.7);old=n.planner.futures[-1]
for attempt in range(3):
    clock[0]+=5.;n.check_path_deadline(Time(seconds=clock[0]))
    if attempt<2:n.request_path(-.74,1.7)
checks['nav2_times_out_and_faults_after_three_attempts']=n.fault is not None and not n.path_pending and n.path_attempts==3
old.fire();checks['late_acceptance_is_cancelled']=old.value.cancelled==1 and n.path is None
clock[0]=200.;n,cmd,status,dec=node_at(2);n.planner=Planner();n.path=None
n.request_path(-.74,1.7);f=n.planner.futures[-1];f.fire();handle=f.value
clock[0]+=5.;n.check_path_deadline(Time(seconds=clock[0]));n.request_path(-.74,1.7)
path=NavPath();path.poses=[PoseStamped()]
handle.future.value=SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED,result=SimpleNamespace(path=path));handle.future.fire()
checks['late_path_cannot_replace_new_request']=n.path is None and n.path_pending and handle.cancelled==1
f=n.planner.futures[-1];f.error=RuntimeError('injected transport error');f.fire()
checks['future_exception_is_bounded_not_uncaught']=not n.path_pending and n.fault is None
n.request_path(-.74,1.7);f=n.planner.futures[-1];f.fire()
f.value.future.value=SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED,result=SimpleNamespace(path=path));f.value.future.fire()
checks['valid_planner_result_recovers']=n.path is path and not n.path_pending and n.path_request_deadline is None
checks['successful_plan_resets_retry_budget']=n.path_attempts==0
clock[0]=300.;n,cmd,status,dec=node_at(2);n.planner=SimpleNamespace(server_is_ready=lambda:False)
for _ in range(3):n.request_path(0.,0.);clock[0]+=5.;n.check_path_deadline(Time(seconds=clock[0]))
checks['absent_planner_server_also_bounded']=n.fault is not None and n.path_attempts==3
report={'passed':all(checks.values()),'checks':checks,'scope':'Actual runtime functions on isolated state, no actuator connection; full physical Gazebo acceptance separate.'}
p=argparse.ArgumentParser();p.add_argument('--output',default='validation/boundary-fixes-2026-10-06/controller-regression.json');a=p.parse_args()
out=Path(__file__).resolve().parents[1]/a.output;out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2));raise SystemExit(0 if report['passed'] else 1)
