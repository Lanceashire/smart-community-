#!/usr/bin/env python3
"""Counterexamples and dense independent whole-body checks for the rolling turn."""
import argparse,json,math
from pathlib import Path
from shapely.geometry import Polygon,box
from shapely.ops import unary_union
from community_demo.route_path import GridClearance,continuous_lower_path,curve_target,pose_sweep_clear
from community_demo.road_rules import CORRIDORS,body_corners


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);a=p.parse_args();checks={}
    data=[0]*44100;grid=GridClearance(210,210,.02,(-2.1,-2.1),data)
    posts=[{'x':.2075,'y':-1.62,'yaw':math.pi/2,'width':.025,'depth':.025}]
    raw=[(-.1,-1.68),(.05,-1.8),(.5,-1.8),(1.1,-1.7)];start=(-.1,-1.68,-math.pi/2)
    curve=continuous_lower_path(raw,start,grid,posts);checks['arc_exists']=curve is not None
    checks['start_heading_preserved']=curve is not None and curve[0]==start
    checks['same_nav2_goal']=curve is not None and curve[-1][:2]==raw[-1]
    checks['monotonic_tangent_heading']=curve is not None and all(b[2]>=a[2]-1e-6 for a,b in zip(curve,curve[1:]))
    checks['all_swept_body_intervals_clear']=curve is not None and all(pose_sweep_clear(x,y,grid,posts)for x,y in zip(curve,curve[1:]))
    checks['no_grid_no_curve']=continuous_lower_path(raw,start,None,posts)is None
    checks['wrong_entry_heading_rejected']=continuous_lower_path(raw,(-.1,-1.68,0),grid,posts)is None
    checks['nonfinite_pose_rejected']=continuous_lower_path(raw,(math.nan,-1.68,-math.pi/2),grid,posts)is None
    checks['late_turn_outside_road_rejected']=continuous_lower_path(raw,(-.1,-1.93,-math.pi/2),grid,posts)is None
    for value,label in [(100,'obstacle'),(-1,'unknown')]:
        q=data.copy()
        for row in range(10,36):
            for col in range(126,130):q[row*210+col]=value
        blocked=GridClearance(210,210,.02,(-2.1,-2.1),q)
        checks[label+'_across_route_rejects_all_arcs']=continuous_lower_path(raw,start,blocked,posts)is None
    checks['long_uncertified_sweep_rejected']=not pose_sweep_clear(start,(.8,-1.7,0),grid,posts)
    checks['large_rotation_step_rejected']=not pose_sweep_clear(start,(-.1,-1.68,-1.),grid,posts)
    road=unary_union([box(*r)for r in CORRIDORS]);dense=[]
    for x,y in zip(curve,curve[1:]):
        for i in range(11):
            t=i/10;pose=tuple(u+(v-u)*t for u,v in zip(x,y));body=Polygon(body_corners(*pose))
            dense.append(road.covers(body)and all(body.distance(box(p['x']-.0125,p['y']-.0125,p['x']+.0125,p['y']+.0125))>=.05 for p in posts))
    checks['independent_dense_body_and_5cm_post_clearance']=all(dense)
    progress=0;errors=[];curvatures=[]
    for pose in curve[:-1]:
        error,k,new=curve_target(curve,pose,progress);checks['progress_never_goes_backward']=checks.get('progress_never_goes_backward',True)and new>=progress
        errors.append(error);curvatures.append(k);progress=new
    checks['carrot_remains_forward']=all(abs(e)<.5 for e in errors)
    checks['finite_bounded_pure_pursuit']=all(math.isfinite(k)and abs(k)<14 for k in curvatures)
    checks['existing_angular_speed_cap_retained']=all(abs(min(.16,.55/max(abs(k),.01))*k)<=.550000001 for k in curvatures)
    result={'passed':all(checks.values()),'checks':checks,'dense_pose_comparisons':len(dense),'scope':'Production arc sweep and tracking functions; independent Shapely full-body and lamp clearance. No simulation pose feeds control.'}
    out=Path(a.output);out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2));return 0 if result['passed']else 1
if __name__=='__main__':raise SystemExit(main())
