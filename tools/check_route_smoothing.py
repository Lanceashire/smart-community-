#!/usr/bin/env python3
"""Counterexamples for shortcut safety and the original corner-containment hole."""
import argparse,json,math,random
from pathlib import Path
from types import SimpleNamespace
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid,Path as NavPath
from geometry_msgs.msg import PoseStamped
from community_demo.road_rules import safe_pose,CORRIDORS,body_corners
from community_demo.route_path import GridClearance,simplify_path,segment_clear,pivot_clear
from community_demo.patrol import Patrol

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);a=p.parse_args();checks={}
    grid=GridClearance(210,210,.02,(-2.1,-2.1),[0]*44100)
    checks['redundant_wiggles_removed']=simplify_path([(-.1,-1.8),(.1,-1.83),(.4,-1.75),(1.1,-1.8)],(-.1,-1.8),grid,[])==[(-.1,-1.8),(1.1,-1.8)]
    checks['no_shortcut_across_inner_block']=not segment_clear((-1.7,.5),(-.1,-1.7),grid,[])
    checks['whole_body_not_only_centre_checked']=not segment_clear((.3,-1.95),(1.1,-1.95),grid,[])
    post={'x':.2075,'y':-1.62,'yaw':math.pi/2,'width':.025,'depth':.025}
    checks['post_margin_blocks_premature_diagonal_to_lane_centre']=not segment_clear((-.1,-1.78),(1.1,-1.7),grid,[post])
    checks['old_5_5cm_pass_rejected_by_larger_planned_margin']=not segment_clear((-.1,-1.8),(.5,-1.8),grid,[post])
    checks['post_clear_lower_pass_permitted']=segment_clear((-.1,-1.825),(.5,-1.825),grid,[post])
    registered=list(grid.data);registered[22*210+116]=100
    measured=GridClearance(210,210,.02,(-2.1,-2.1),registered)
    crooked=[(-.1,-1.825),(.1,-1.84),(.36,-1.844),(.42,-1.775),(.7,-1.775),(1.1,-1.7)]
    staged=simplify_path(crooked,crooked[0],measured,[post])
    checks['unsafe_gradient_bend_gets_checked_staging_point']=bool(staged) and len(staged)==3 and staged[1][1]==-1.825
    checks['both_staged_sweeps_and_pivot_remain_clear']=bool(staged) and all(segment_clear(x,y,measured,[post]) for x,y in zip(staged,staged[1:])) and pivot_clear(staged[1],measured,[post])
    checks['pivot_near_outer_line_rejected']=not pivot_clear((.5,-1.85),grid,[])
    checks['pivot_at_junction_has_full_rotation_clearance']=pivot_clear((1.1,-1.7),grid,[])
    for value,label in [(100,'lethal'),(-1,'unknown')]:
        changed=list(grid.data);changed[15*210+130]=value
        blocked=GridClearance(210,210,.02,(-2.1,-2.1),changed)
        checks[label+'_cell_in_swept_body_rejects_shortcut']=not segment_clear((-.1,-1.8),(1.1,-1.8),blocked,[])
    checks['outside_grid_rejected']=not grid.clear([(-3,0),(-2.5,0),(-2.5,.1),(-3,.1)])
    checks['no_grid_no_simplification']=simplify_path([(0,0),(1,0)],(0,0),None,[]) is None
    checks['concave_triangle_008_rejected']=not safe_pose(-1.4697116942902504,.7149493560864425,.9861290046104765)
    checks['concave_triangle_004_rejected']=not safe_pose(.8929902713403237,-1.460988118150391,.8576998359556547,margin=.004)
    checks['nonfinite_pose_rejected']=not safe_pose(math.nan,1.7,0.)
    node=SimpleNamespace(costmap=None,costmap_stamp_ns=None,get_clock=lambda:SimpleNamespace(now=lambda:Time(seconds=20.)))
    m=OccupancyGrid();m.header.frame_id='field';m.header.stamp=Time(seconds=19.5).to_msg()
    m.info.width=m.info.height=210;m.info.resolution=.02;m.info.origin.position.x=m.info.origin.position.y=-2.1;m.info.origin.orientation.w=1.;m.data=[0]*44100
    Patrol.on_costmap(node,m);checks['fresh_field_costmap_accepted']=node.costmap is not None
    first=node.costmap;m.header.stamp=Time(seconds=19.6).to_msg();Patrol.on_costmap(node,m)
    checks['unchanged_costs_with_fresh_source_refresh_grid']=node.costmap is not first and node.costmap_stamp_ns==19_600_000_000
    for stamp,label in [(10.,'stale'),(21.,'future'),(19.4,'out_of_order')]:
        old=node.costmap;m.header.stamp=Time(seconds=stamp).to_msg();Patrol.on_costmap(node,m)
        checks[label+'_costmap_cannot_replace_current']=node.costmap is old
    m.header.stamp=Time(seconds=20.).to_msg();m.header.frame_id='map';old=node.costmap;Patrol.on_costmap(node,m)
    checks['wrong_frame_rejected']=node.costmap is old
    m.header.frame_id='field';m.info.origin.orientation.z=.1;Patrol.on_costmap(node,m)
    checks['rotated_grid_rejected']=node.costmap is old
    m.info.origin.orientation.z=0.;m.data=[0];Patrol.on_costmap(node,m)
    checks['truncated_grid_rejected']=node.costmap is old
    # The continuous tracker follows the certified curve without reverting to
    # the old stop-and-pivot staging algorithm or moving its progress backwards.
    from community_demo.trajectory import plan_trajectory
    curve,meta=plan_trajectory([(-1.26,1.7),(-1.7,1.04)],(-1.26,1.7,math.pi),-math.pi/2,grid,[])
    path=NavPath()
    for x,y,yaw in curve:
        q=PoseStamped();q.pose.position.x=x;q.pose.position.y=y;path.poses.append(q)
    n=SimpleNamespace(path=path,curve_progress=0,path_goal_yaw=-math.pi/2,
                      pose=curve[40],get_clock=node.get_clock)
    error=Patrol.path_heading_error(n)
    checks['continuous_tracker_has_positive_remaining_arc']=math.isfinite(error) and n.path_remaining_arc>0 and n.curve_progress>=39
    previous=n.curve_progress;n.pose=curve[25];Patrol.path_heading_error(n)
    checks['curve_progress_never_rewinds']=n.curve_progress>=previous
    # Independent geometry library is test-only, never a runtime dependency.
    from shapely.geometry import Polygon,box
    from shapely.ops import unary_union
    road=unary_union([box(x+.008,y+.008,z-.008,w-.008) for x,y,z,w in CORRIDORS])
    rng=random.Random(20261010);mismatches=[]
    for _ in range(3000):
        pose=(rng.uniform(-2.1,2.1),rng.uniform(-2.1,2.1),rng.uniform(-math.pi,math.pi))
        if safe_pose(*pose)!=road.covers(Polygon(body_corners(*pose))):mismatches.append(pose)
    checks['independent_polygon_union_3000_cases']=not mismatches
    result={'passed':all(checks.values()),'checks':checks,'polygon_comparisons':3000,
            'scope':'Production geometry, live-grid rejection and path tracking; independent polygon comparison. Physical turning and perception checked in live Gazebo separately.'}
    out=Path(a.output);out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2));return 0 if result['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
