#!/usr/bin/env python3
"""Negative and positive measurement evidence for sparse laser-map outliers."""
import argparse,json,math
from pathlib import Path
import numpy as np
from community_demo.mapping_grid import free_ray_indices,fuse_registered_rays,SIZE

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);a=p.parse_args();checks={}
    def cell(x,y):return int(math.floor((y+2.1)/.02))*210+int(math.floor((x+2.1)/.02))
    rays=free_ray_indices((0.,0.),[0.,0.],[1.,1.],.17,6.)
    checks['one_vote_per_cell_per_frame_despite_duplicate_beams']=len(rays)==len(set(rays))
    checks['known_free_before_return']=cell(.5,0.) in rays
    checks['finite_endpoint_not_declared_free']=cell(1.,0.) not in rays
    checks['past_finite_return_not_declared_free']=cell(1.2,0.) not in rays
    checks['sensor_blind_region_not_declared_free']=cell(.05,0.) not in rays
    no_return=free_ray_indices((0.,0.),[0.],[math.inf],.17,6.)
    checks['infinite_return_confirms_only_bounded_observed_ray']=cell(1.5,0.) in no_return and all(0<=i<SIZE*SIZE for i in no_return)
    rotated=free_ray_indices((.1,.2),[math.pi/2],[1.],.17,6.)
    checks['rays_registered_with_actual_field_angle_and_origin']=cell(.1,.7) in rotated and cell(.7,.2) not in rotated
    for ranges,name in [([math.nan],'nan'),([-math.inf],'negative_inf'),([0.],'below_range'),([7.],'above_range')]:
        try:free_ray_indices((0.,0.),[0.],ranges,.17,6.);checks[name+'_rejected']=False
        except ValueError:checks[name+'_rejected']=True
    slam=np.full((210,210),-1,np.int16);hits=np.zeros_like(slam,np.uint16);free=np.zeros_like(hits)
    checks['unseen_unknown_preserved']=np.array_equal(fuse_registered_rays(slam,hits,free),slam)
    hits[192,90]=4;slam[192,90]=100;free[192,90]=11
    checks['less_than_three_clear_votes_per_hit_cannot_erase']=fuse_registered_rays(slam,hits,free)[192,90]==100
    free[192,90]=12
    checks['sparse_four_hit_outlier_clears_with_twelve_independent_free_frames']=fuse_registered_rays(slam,hits,free)[192,90]==0
    hits[174,90]=331;free[174,90]=300
    checks['persistent_physical_post_retained_despite_partial_cell_free_votes']=fuse_registered_rays(slam,hits,free)[174,90]==100
    free[100,100]=2
    checks['two_frames_do_not_fill_unknown']=fuse_registered_rays(slam,hits,free)[100,100]==-1
    free[100,100]=3
    checks['three_actual_free_ray_frames_can_resolve_unknown']=fuse_registered_rays(slam,hits,free)[100,100]==0
    hits[150,150]=65535;free[150,150]=65535
    checks['vote_ratio_does_not_overflow_uint16']=fuse_registered_rays(slam,hits,free)[150,150]==100
    checks={k:bool(v) for k,v in checks.items()}
    result={'passed':all(checks.values()),'checks':checks,'scope':'Ray source, range, uniqueness, unknown preservation and contradictory occupancy evidence. Actual mapped navigation and physical posts checked separately.'}
    out=Path(a.output);out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2));return 0 if result['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
