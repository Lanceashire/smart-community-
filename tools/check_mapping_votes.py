#!/usr/bin/env python3
"""Counterexamples for new obstacles, clear rays, vote ordering and saturation."""
import argparse,json,sys
from pathlib import Path
from types import SimpleNamespace as S
import numpy as np
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from community_demo.mapping_grid import update_registered_votes,fuse_registered_rays
sys.path.insert(0,str(Path(__file__).resolve().parent))
from collect_mapping_evidence import Capture

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);a=p.parse_args();checks={}
    h=np.zeros((210,210),np.uint16);f=h.copy();s=np.zeros((210,210),np.int16);index=100;f.ravel()[index]=100;s.ravel()[index]=100
    for _ in range(3):update_registered_votes(h,f,[index,index],[index,index+1])
    checks.update(new_obstacle_resets_historical_frees=f.ravel()[index]==0,
                  new_three_hit_obstacle_stays_occupied=fuse_registered_rays(s,h,f).ravel()[index]==100,
                  duplicate_and_same_frame_ray_cannot_override_endpoint=h.ravel()[index]==3 and f.ravel()[index]==0,
                  free_votes_unique_per_scan=f.ravel()[index+1]==3)
    s.ravel()[index]=100
    for _ in range(8):update_registered_votes(h,f,[],[index])
    checks['insufficient_new_clear_rays_keep_obstacle']=fuse_registered_rays(s,h,f).ravel()[index]==100
    update_registered_votes(h,f,[],[index]);checks['repeated_clear_after_hit_removes_ghost']=fuse_registered_rays(s,h,f).ravel()[index]==0
    update_registered_votes(h,f,[index],[]);checks['later_reappearance_is_not_erased']=fuse_registered_rays(s,h,f).ravel()[index]==100
    h.ravel()[index]=f.ravel()[index+1]=65535;update_registered_votes(h,f,[index],[index+1])
    checks['uint16_votes_saturate']=h.ravel()[index]==65535 and f.ravel()[index+1]==65535
    def scan(t):
        m=LaserScan();m.header.stamp=Time(nanoseconds=t).to_msg();m.angle_min=0.;m.angle_increment=.01;m.ranges=[1.];m.range_min=.17;m.range_max=6.;return m
    old=scan(100_000_000);new=scan(300_000_000)
    c=S(pending_scans=[new,old],last_registered_ns=200_000_000,get_clock=lambda:S(now=lambda:Time(nanoseconds=300_000_000)),
        tf=S(lookup_transform=lambda *a:S(transform=S(rotation=S(x=0.,y=0.,z=0.,w=1.),translation=S(x=0.,y=0.)))),
        hit_counts=np.zeros((210,210),np.uint16),free_counts=np.zeros((210,210),np.uint16),registered_scans=0)
    Capture.register_hits(c)
    checks['older_source_scan_cannot_rewrite_votes']=c.registered_scans==1 and int(c.hit_counts.sum())==1 and c.last_registered_ns==300_000_000
    c.pending_scans=[new];Capture.register_hits(c);checks['duplicate_source_scan_cannot_vote_twice']=c.registered_scans==1
    result={'passed':bool(all(checks.values())),'checks':{k:bool(v) for k,v in checks.items()},'scope':'Production vote fusion and actual collector callback, with explicit synthetic source-stamped scans.'}
    out=Path(a.output);out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result));return 0 if result['passed']else 1

if __name__=='__main__':raise SystemExit(main())
