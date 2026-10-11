import math,json,sys
from pathlib import Path
from shapely.geometry import Polygon,box
from shapely.ops import unary_union
from community_demo.route_path import GridClearance,continuous_corner_path,pose_sweep_clear,continuous_lower_path,continuous_return_path,terminal_sweep_clear,curve_target
from community_demo.road_rules import CORRIDORS,body_corners
root=Path(__file__).resolve().parents[1];import argparse
p=argparse.ArgumentParser();p.add_argument('--output',required=True);a=p.parse_args();out=Path(a.output)
grid=GridClearance(210,210,.02,(-2.1,-2.1),[0]*44100)
cases={'top_west':((-1.54,1.7,math.pi),(-1.7,1.04),-math.pi/2),
       'left_middle':((-1.7,.66,-math.pi/2),(-.9,.5),0.),
       'middle_down':((-.26,.5,0.),(-.1,-.09),-math.pi/2)}
checks={};comparisons=0;road=unary_union([box(*r)for r in CORRIDORS])
posts=json.loads((root/'src/community_demo/maps/planning-obstacles.json').read_text())['posts']
for name,(pose,goal,yaw)in cases.items():
    points=[pose[:2],goal];curve=continuous_corner_path(points,pose,yaw,grid,posts)
    checks[name+'_curve_exists']=curve is not None
    checks[name+'_same_goal_and_measured_start']=bool(curve)and math.dist(curve[0][:2],pose[:2])<1e-8 and math.dist(curve[-1][:2],goal)<1e-8
    checks[name+'_exit_tangent']=bool(curve)and abs(math.atan2(math.sin(curve[-1][2]-yaw),math.cos(curve[-1][2]-yaw)))<1e-8
    checks[name+'_whole_sweeps_clear']=bool(curve)and all(pose_sweep_clear(a,b,grid,posts)for a,b in zip(curve,curve[1:]))
    valid=[]
    for a,b in zip(curve,curve[1:]):
        for i in range(11):
            t=i/10;z=tuple(x+(y-x)*t for x,y in zip(a,b));body=Polygon(body_corners(*z));valid.append(road.covers(body)and all(body.distance(box(p['x']-.0125,p['y']-.0125,p['x']+.0125,p['y']+.0125))>=.05 for p in posts));comparisons+=1
    checks[name+'_independent_dense_body_and_post_margin']=all(valid)
    for value,label in [(100,'occupied'),(-1,'unknown')]:
        q=grid.data.copy();middle=curve[len(curve)//3];row=int((middle[1]+2.1)/.02);col=int((middle[0]+2.1)/.02);q[row*210+col]=value
        checks[name+'_'+label+'_cell_rejects_curve']=continuous_corner_path(points,pose,yaw,GridClearance(210,210,.02,(-2.1,-2.1),q),posts)is None
checks['no_grid_rejected']=continuous_corner_path([(-1.54,1.7),(-1.7,1.04)],(-1.54,1.7,math.pi),-math.pi/2,None,posts)is None
checks['nonfinite_pose_rejected']=continuous_corner_path([(-1.54,1.7),(-1.7,1.04)],(math.nan,1.7,math.pi),-math.pi/2,grid,posts)is None
checks['late_entry_small_radius_rejected']=continuous_corner_path([(-1.68,1.7),(-1.7,1.04)],(-1.68,1.7,math.pi),-math.pi/2,grid,posts)is None
checks['wrong_heading_rejected']=continuous_corner_path([(-1.54,1.7),(-1.7,1.04)],(-1.54,1.7,0),-math.pi/2,grid,posts)is None
for name,pose,goal,fn in [
    ('plate_arrival',(-.1,-1.67,-math.pi/2),(1.1,-1.7),lambda p,z,g,b:continuous_lower_path(p,z,g,b,arrival_turn=True)),
    ('return_arrival',(1.1,-.53,math.pi/2),(1.1,1.7),continuous_return_path)]:
    curve=fn([pose[:2],goal],pose,grid,posts)
    checks[name+'_curve_exists']=curve is not None
    if curve is None:continue
    checks[name+'_no_coordinate_jump']=all(math.dist(a[:2],b[:2])<=.016 for a,b in zip(curve,curve[1:]))
    checks[name+'_same_goal']=math.dist(curve[-1][:2],goal)<1e-8
    expected_yaw=math.pi/2 if name=='plate_arrival' else math.pi
    checks[name+'_required_terminal_heading']=abs(math.atan2(math.sin(curve[-1][2]-expected_yaw),math.cos(curve[-1][2]-expected_yaw)))<1e-8
    checks[name+'_start_matches_measured_heading']=abs(math.atan2(math.sin(curve[0][2]-pose[2]),math.cos(curve[0][2]-pose[2])))<1e-8
    checks[name+'_certified_tangent_extension']=terminal_sweep_clear(curve[-1],grid,posts)
    checks[name+'_whole_sweeps_clear']=all(pose_sweep_clear(a,b,grid,posts)for a,b in zip(curve,curve[1:]))
    dense=[]
    for a,b in zip(curve,curve[1:]):
        for i in range(11):
            t=i/10;z=tuple(x+(y-x)*t for x,y in zip(a,b));body=Polygon(body_corners(*z));dense.append(road.covers(body)and all(body.distance(box(p['x']-.0125,p['y']-.0125,p['x']+.0125,p['y']+.0125))>=.065 for p in posts));comparisons+=1
    checks[name+'_independent_dense_body_and_6_5cm_margin']=all(dense)
    points=[p[:2]for p in curve];idx=len(points)//4;z=curve[idx];reference=curve_target(points,z,idx,curve[-1][2])
    def rotate(p,theta):return p[0]*math.cos(theta)-p[1]*math.sin(theta),p[0]*math.sin(theta)+p[1]*math.cos(theta)
    for theta in (math.pi/2,-math.pi/2,math.pi):
        transformed=[rotate(p,theta)for p in points];xy=rotate(z,theta);actual=curve_target(transformed,(*xy,z[2]+theta),idx,curve[-1][2]+theta)
        checks[name+'_tracking_rotation_invariant_'+str(theta)]=all(abs(a-b)<1e-8 for a,b in zip(reference,actual))
    q=grid.data.copy();end=curve[-1];px=end[0]+.20*math.cos(end[2]);py=end[1]+.20*math.sin(end[2]);row=int((py+2.1)/.02);col=int((px+2.1)/.02);q[row*210+col]=-1
    checks[name+'_unknown_tangent_extension_rejected']=fn([pose[:2],goal],pose,GridClearance(210,210,.02,(-2.1,-2.1),q),posts)is None
from community_demo.route_path import segment_clear
checks['old_5_5cm_planned_pass_now_rejected']=not segment_clear((-.1,-1.8),(.5,-1.8),grid,posts)
result={'passed':all(checks.values()),'checks':checks,'dense_body_comparisons':comparisons,'scope':'Production three-corner arc function, fixed task route prior, independent dense complete-body/post comparison; live execution audited separately.'}
out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({'passed':result['passed'],'failed':[k for k,v in checks.items()if not v],'checks':len(checks),'dense_body_comparisons':comparisons}));raise SystemExit(0 if result['passed']else 1)
