#!/usr/bin/env python3
"""Safety counterexamples for measured-map projection and lane composition."""
import json,math
from types import SimpleNamespace as S
import numpy as np
from community_demo.mapping_grid import field_grid,navigation_grid,SIZE
q=S(x=0,y=0,z=0,w=1)
def msg(a,origin=(-2.1,-2.1),rotation=q):return S(info=S(width=a.shape[1],height=a.shape[0],resolution=.02,origin=S(position=S(x=origin[0],y=origin[1]),orientation=rotation)),data=a.ravel().tolist())
checks={}
a=np.zeros((210,210),np.int16);a[100,150]=100;a[85,90]=-1
out=field_grid(msg(a),(0,0,0));checks['identity_keeps_measured_occupied_and_unknown']=np.array_equal(a,out)
out=field_grid(msg(a),(0,0,math.pi));checks['rotation_transforms_measured_obstacle']=out[109,59]==100 and out[124,119]==-1
out=field_grid(msg(a),(100,100,0));checks['outside_source_stays_unknown']=np.all(out==-1)
unknown=np.full((210,210),-1,np.int16);nav,e=navigation_grid(unknown);checks['unknown_road_never_filled_free']=not (nav==0).any() and e['known_road_fraction']==0
free=np.zeros((210,210),np.int16);nav,e=navigation_grid(free);checks['provided_lane_kept_separate']=nav[130,130]==100 and nav[160,160]==0
free[140,160]=100;nav,e=navigation_grid(free);checks['new_measured_obstacle_is_in_navigation']=nav[140,160]==100 and nav[141,161]==100 and e['measured_occupied_road_cells']==1
try:field_grid(S(info=S(width=1,height=1,resolution=0,origin=S(position=S(x=0,y=0),orientation=q)),data=[0]),(0,0,0));checks['invalid_resolution_rejected']=False
except ValueError:checks['invalid_resolution_rejected']=True
checks={k:bool(v) for k,v in checks.items()}
r={'passed':all(checks.values()),'checks':checks};print(json.dumps(r,indent=2));raise SystemExit(0 if r['passed'] else 1)
