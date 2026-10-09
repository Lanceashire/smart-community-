#!/usr/bin/env python3
"""Actual IMU and mapping-scan callbacks on isolated publishers; no actuators."""
import argparse,copy,json,math
from collections import deque
from pathlib import Path
from types import SimpleNamespace as S
from rclpy.time import Time
from sensor_msgs.msg import Imu,LaserScan
from community_demo.imu_heading import imu_sample,heading_at
from community_demo.localization import Localization
from community_demo.slam_scan import Adapter,mapping_scan

checks={};clock=[10.]
def imu(t=10.,yaw=.2):
    m=Imu();m.header.stamp=Time(seconds=t).to_msg()
    m.orientation.z=math.sin(yaw/2);m.orientation.w=math.cos(yaw/2)
    m.angular_velocity.z=.1;return m
m=imu();sample=imu_sample(m)
checks['valid_source_stamped_heading']=sample is not None and abs(sample[1]-.2)<1e-9
for name,change in [('zero_quaternion',lambda m:setattr(m.orientation,'w',0.)),
                    ('nan_heading',lambda m:setattr(m.orientation,'z',math.nan)),
                    ('nan_rate',lambda m:setattr(m.angular_velocity,'z',math.nan)),
                    ('orientation_unavailable',lambda m:m.orientation_covariance.__setitem__(0,-1.))]:
    bad=copy.deepcopy(m);change(bad)
    if name=='zero_quaternion':bad.orientation.z=0.
    checks[name+'_rejected']=imu_sample(bad) is None
checks['heading_extrapolates_only_short_source_interval']=abs(heading_at([sample],10.04)-.204)<1e-9 and heading_at([sample],10.06) is None
checks['heading_wraps_at_pi']=abs(heading_at([(10.,math.pi-.001,.1)],10.04)-(-math.pi+.003))<1e-9
node=S(get_clock=lambda:S(now=lambda:Time(seconds=clock[0])),imu_samples=deque(maxlen=512),imu_reference=None)
Localization.imu(node,m);checks['known_start_calibration']=len(node.imu_samples)==1 and abs(abs(node.imu_samples[0][1])-math.pi)<1e-9
Localization.imu(node,m);checks['replayed_imu_rejected']=len(node.imu_samples)==1
Localization.imu(node,imu(10.01));Localization.imu(node,imu(9.8));checks['future_and_stale_imu_rejected']=len(node.imu_samples)==1
clock[0]=10.1;Localization.imu(node,imu(10.1,.3));checks['relative_sensor_turn_used']=len(node.imu_samples)==2 and abs(node.imu_samples[-1][1]-(-math.pi+.1))<1e-9

quality=[];status=[];poses=[]
node.require_imu=True;node.pending_image=None;node.wheel_samples=[]
node.quality=S(publish=lambda m:quality.append(json.loads(m.data)))
node.status=S(publish=lambda m:status.append(m.data));node.poses=S(publish=poses.append)
clock[0]=10.3;Localization.publish(node)
checks['stale_imu_withholds_navigation_pose']=not poses and quality[-1]['state']=='lost' and quality[-1]['reason']=='IMU source freshness'
node.imu_samples.clear();Localization.publish(node)
checks['missing_imu_withholds_navigation_pose']=not poses and quality[-1]['state']=='lost'

scan=LaserScan();scan.header.stamp=Time(seconds=10.).to_msg();scan.angle_min=-math.pi;scan.angle_increment=math.pi/360
scan.angle_max=scan.angle_min+719*scan.angle_increment;scan.range_min=.17;scan.range_max=6.;scan.ranges=[math.inf]*720;scan.ranges[360]=.3
result=mapping_scan(scan)
checks['finite_obstacle_beam_preserved']=result.ranges[360]==scan.ranges[360]
checks['no_return_only_frees_threshold_ray']=all(abs(r-5.99)<1e-5 for i,r in enumerate(result.ranges) if i!=360) and 5.95<result.ranges[0]<scan.range_max
checks['collision_scan_never_mutated']=math.isinf(scan.ranges[0]) and result is not scan
bad=copy.deepcopy(scan);bad.ranges[0]=math.nan;checks['nan_mapping_scan_rejected']=mapping_scan(bad) is None
published=[];clock[0]=10.
adapter=S(previous=None,get_clock=node.get_clock,pub=S(publish=published.append))
Adapter.scan(adapter,scan);Adapter.scan(adapter,scan)
checks['same_scan_published_once']=len(published)==1
bad=copy.deepcopy(scan);bad.header.stamp=Time(seconds=9.).to_msg();Adapter.scan(adapter,bad)
bad.header.stamp=Time(seconds=10.1).to_msg();Adapter.scan(adapter,bad)
checks['old_and_future_scan_rejected']=len(published)==1
report={'passed':all(checks.values()),'checks':checks,'scope':'Actual production functions, isolated outputs; Gazebo IMU noise and real hardware calibration remain untested.'}
p=argparse.ArgumentParser();p.add_argument('--output',required=True);a=p.parse_args();out=Path(a.output);out.parent.mkdir(parents=True,exist_ok=True)
out.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2));raise SystemExit(0 if report['passed'] else 1)
