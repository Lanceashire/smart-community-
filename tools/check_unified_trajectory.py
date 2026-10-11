#!/usr/bin/env python3
"""Independent footprint checks, changed layouts, and command counterexamples."""
import argparse,ast,json,math
from pathlib import Path
import numpy as np
from shapely.geometry import Polygon,box
from shapely.ops import unary_union
from community_demo.trajectory import plan_trajectory
from community_demo.route_path import GridClearance,curve_target,rolling_terminal_allowed
from community_demo.road_rules import CORRIDORS,body_corners
from community_demo.motion_profile import path_velocity,velocity_envelope,coupled_command

ROOT=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser();parser.add_argument('--output',required=True);args=parser.parse_args()
posts=json.loads((ROOT/'src/community_demo/maps/planning-obstacles.json').read_text())['posts']
road=unary_union([box(*r)for r in CORRIDORS]);checks={};dense=0;cases=[]
specs=[('top',(-1.26,1.7,math.pi),(-1.7,1.04,-math.pi/2)),
 ('left',(-1.7,.94,-math.pi/2),(-.9,.5,0)),('middle',(-.54,.5,0),(-.1,-.09,-math.pi/2)),
 ('lower',(-.1,-1.676,-math.pi/2),(1.1,-1.7,math.pi/2)),
 ('finish',(1.1,-.53,math.pi/2),(1.7,1.7,0.0))]
grid=GridClearance(210,210,.02,(-2.1,-2.1),[0]*44100)
for name,start,goal in specs:
    points=[start[:2],(start[0],goal[1]),goal[:2]] if name=='finish' else [start[:2],goal[:2]];curve,meta=plan_trajectory(points,start,goal[2],grid,posts)
    checks[name+'_certified_curve_exists']=curve is not None
    if curve is None:continue
    checks[name+'_exact_start_and_goal']=math.dist(curve[0][:2],start[:2])<1e-8 and math.dist(curve[-1][:2],goal[:2])<1e-5
    checks[name+'_both_endpoint_headings']=abs(math.atan2(math.sin(curve[0][2]-start[2]),math.cos(curve[0][2]-start[2])))<1e-8 and abs(math.atan2(math.sin(curve[-1][2]-goal[2]),math.cos(curve[-1][2]-goal[2])))<1e-5
    valid=[]
    for a,b in zip(curve,curve[1:]):
        for i in range(6):
            z=tuple(x+(y-x)*i/5 for x,y in zip(a,b));body=Polygon(body_corners(*z));dense+=1
            valid.append(road.covers(body) and body.distance(road.boundary)>=.008 and all(body.distance(box(p['x']-.0125,p['y']-.0125,p['x']+.0125,p['y']+.0125))>=.065 for p in posts))
    checks[name+'_independent_dense_full_body_road_and_posts']=all(valid)
    env=velocity_envelope(curve);k=env['curvature'];v=env['speed_m_s']
    checks[name+'_profile_angular_speed']=max(abs(a*b)for a,b in zip(k,v))<=.65+1e-8
    dt=2*np.diff(env['arc_m'])/np.maximum(np.asarray(v[:-1])+np.asarray(v[1:]),1e-9)
    checks[name+'_profile_angular_acceleration']=bool(max(abs(np.diff(np.asarray(k)*np.asarray(v))/dt))<=1.2+1e-8)
    checks[name+'_profile_lateral_acceleration']=max(abs(a)*b*b for a,b in zip(k,v))<=.20+1e-8
    checks[name+'_endpoint_curvature_tends_to_zero']=max(abs(k[0]),abs(k[-1]))<1.1
    if name=='finish':
        checks['finish_same_start_coordinate']=math.dist(curve[-1][:2],(1.7,1.7))<1e-5
        checks['finish_long_straight_centre_under_2mm']=all(abs(z[0]-1.1)<=.002 for z in curve if -.3<=z[1]<=.6)
    cases.append({'name':name,'meta':meta,'curve_points':len(curve)})
    # Translate, reflect, and rotate the entire constrained scene; controller
    # parameters and solver code are unchanged, including the full footprint.
    for label,angle,mirror in [('rotate90',math.pi/2,False),('translate',0,False),('reflect',0,True)]:
        def xy(z):
            x,y=z[:2]
            if mirror:x=-x
            return x*math.cos(angle)-y*math.sin(angle)+.3,x*math.sin(angle)+y*math.cos(angle)-.4
        def yaw(h):return (math.pi-h if mirror else h)+angle
        roads=[]
        for a,b,c,d in CORRIDORS:
            vertices=[xy(z)for z in ((a,b),(a,d),(c,b),(c,d))];roads.append((min(z[0]for z in vertices),min(z[1]for z in vertices),max(z[0]for z in vertices),max(z[1]for z in vertices)))
        transformed_posts=[{**p,'x':xy((p['x'],p['y']))[0],'y':xy((p['x'],p['y']))[1],'yaw':yaw(p['yaw'])}for p in posts]
        g=GridClearance(400,400,.02,(-4,-4),[0]*160000,roads)
        other,details=plan_trajectory([xy(z)for z in points],(*xy(start),yaw(start[2])),yaw(goal[2]),g,transformed_posts)
        checks[name+'_'+label+'_same_geometric_solution']=other is not None and len(other)==len(curve) and max(math.dist(xy(a),b[:2]) for a,b in zip(curve,other))<1e-5
        i=len(curve)//3;reference=curve_target(curve,curve[i],i,goal[2])
        transformed=[(*xy(z),yaw(z[2]))for z in curve]
        actual=curve_target(transformed,transformed[i],i,yaw(goal[2]));sign=-1 if mirror else 1
        checks[name+'_'+label+'_tracking_invariant']=abs(reference[0]*sign-actual[0])<1e-8 and abs(reference[1]*sign-actual[1])<1e-8 and reference[2]==actual[2]

wide=GridClearance(400,400,.02,(-4,-4),[0]*160000,[(-3,-3,3,3)])
for i,goal in enumerate([(2,1,math.pi/3),(1.3,-.8,-math.pi/4),(.7,1.5,math.pi*2/3)]):
    c,m=plan_trajectory([(0,0),(goal[0],goal[1])],(0,0,0),goal[2],wide,[])
    checks['different_length_angle_layout_'+str(i)]=c is not None and math.dist(c[-1][:2],goal[:2])<1e-5
checks['no_grid_stops']=plan_trajectory([(0,0),(1,0)],(0,0,0),0,None,[])[0] is None
checks['nonfinite_geometry_stops']=plan_trajectory([(0,0),(math.nan,0)],(0,0,0),0,wide,[])[0] is None
checks['nonfinite_heading_stops']=plan_trajectory([(0,0),(1,0)],(0,0,0),math.inf,wide,[])[0] is None
for value,label in [(100,'occupied'),(-1,'unknown')]:
    data=wide.data.copy()
    for row in range(400):
        for col in range(235,240):data[row*400+col]=value
    blocked=GridClearance(400,400,.02,(-4,-4),data,[(-3,-3,3,3)])
    checks[label+'_full_road_barrier_rejected']=plan_trajectory([(0,0),(1.5,0)],(0,0,0),0,blocked,[])[0] is None
line=[(i*.01,0)for i in range(200)]
checks['straight_cruise']=path_velocity(line,0,0,1)[0]==.26
checks['tight_curvature_speed_cap']=path_velocity(line,0,10,1)[0]<=.065
checks['latency_in_braking_distance']=path_velocity(line,0,0,.08)[0]<math.sqrt(.6*.075)
checks['invalid_geometry_stops']=all(path_velocity(p,i,k,rem)[0]==0 for p,i,k,rem in [([],0,0,1),(line,0,math.nan,1),(line,0,0,-1),(line,len(line)-1,0,1),([(0,0),(0,0)],0,0,1)])
previous=(0.,0.);accel=[]
for desired,k in [( .26,0)]*20+[(.22,3)]*20+[(.08,-7)]*20+[(0,0)]*20:
    current=coupled_command(desired,k,previous,.1);accel.append((abs(current[0]-previous[0])/.1,abs(current[1]-previous[1])/.1));previous=current
checks['coupled_linear_acceleration_limit']=max(a for a,b in accel)<=.5+1e-9
checks['coupled_angular_acceleration_limit']=max(b for a,b in accel)<=1.2+1e-9
checks['terminal_time_bounded']=not rolling_terminal_allowed((1.11,1.7,math.pi-.1),(1.1,1.7),math.pi,.02,grid,posts,1.2)
# Second-lap Gazebo failure: almost at a narrow waypoint with heading still
# off by four degrees, the goal-braking cap was 5mm/s. Rolling cannot finish
# the heading before leaving the arrival circle; stop and pivot instead.
checks['terminal_creep_cannot_escape_arrival_circle']=not rolling_terminal_allowed(
    (-.5433,.5006,.0674),(-.54,.5),0.,.008,grid,posts,0.,speed=.00516,curvature=-2.013)
checks['terminal_wrong_direction_rejected']=not rolling_terminal_allowed(
    (-.5433,.5006,.0674),(-.54,.5),0.,.008,grid,posts,0.,speed=.00516,curvature=2.013)
source=(ROOT/'src/community_demo/community_demo/patrol.py').read_text()
checks['no_named_geometry_templates']=all(s not in source for s in ('continuous_lower_path','continuous_return_path','CONTINUOUS_CORNERS','bend_limit'))
checks['single_solver_and_speed_entry']=source.count('plan_trajectory(points,')==1 and source.count('linear,limits=path_velocity(')==1
task=json.loads((ROOT/'src/community_demo/config/patrol_task.json').read_text())
checks['task_start_finish_identical']=task['start'][:2]==task['finish'][:2]
# Counterexample observed in Gazebo: Nav2 smoothing drifts toward a side goal
# early while the current lane tangent is clear. It must not move the long
# preceding straight, and the endpoint remains the configured start bay.
start=(1.1,-.5,math.pi/2)
reference=[(1.1+.12*min(1.,i/25),-.5+i*.01)for i in range(185)]+[(1.7,1.7)]
c,m=plan_trajectory(reference,start,0.,grid,posts,task['trajectory'])
checks['smoothed_side_goal_keeps_checked_straight']=c is not None and max(abs(z[0]-1.1)for z in c if -.3<=z[1]<=.6)<=.002
checks['side_goal_still_returns_to_same_square']=c is not None and math.dist(c[-1][:2],task['start'][:2])<1e-5
# Measured disturbed entry heading from the failed 2.055cm straight case must
# not be preserved over the long preceding lane. Use the supplied corridor
# centre, with the same exact goal and independent footprint checks above.
entry=(1.10064,-.50696,1.54856)
c,m=plan_trajectory([(1.10064,-.50696),(1.22,1.3),(1.7,1.7)],entry,0.,grid,posts,task['trajectory'])
checks['entry_heading_noise_does_not_accumulate_on_straight']=c is not None and max(
    abs(z[0]-1.1)for z in c if -.3<=z[1]<=.6)<=.002
result={'passed':all(checks.values()),'checks':checks,'dense_body_comparisons':dense,'cases':cases,'scope':'Offline independent geometric scene transformations and alternate configured road layouts; physical fixed-field Gazebo runs are checked separately. No claim of unknown-environment autonomy.'}
out=Path(args.output);out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({'passed':result['passed'],'checks':len(checks),'failed':[k for k,v in checks.items()if not v],'dense':dense,'cases':cases}))
raise SystemExit(0 if result['passed']else 1)
