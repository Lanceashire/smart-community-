#!/usr/bin/env python3
"""Independent read-only proof that Nav2 consumes the newly built grid."""
import argparse,hashlib,json,time
from pathlib import Path
import cv2,numpy as np,rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from nav_msgs.msg import OccupancyGrid,Path as NavPath
from std_msgs.msg import String
from community_demo.patrol import ROUTE,WAYPOINTS
class Verify(Node):
 def __init__(self,mapfile,out):
  super().__init__('mapped_navigation_evidence');self.out=out;self.mapfile=mapfile;self.loaded=None;self.costmaps=[];self.paths=[];self.completed=None;self.status='';self.error=None
  p=cv2.imread(str(mapfile.with_suffix('.pgm')),cv2.IMREAD_GRAYSCALE)
  if p is None:raise ValueError('Missing generated PGM')
  self.expected=np.flipud(np.where(p==0,100,np.where(p>=254,0,-1))).astype(np.int16)
  self.create_subscription(OccupancyGrid,'/field_map',self.on_map,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
  self.create_subscription(OccupancyGrid,'/global_costmap/costmap',self.on_costmap,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
  self.create_subscription(NavPath,'/patrol/path',self.on_path,20)
  self.create_subscription(String,'/patrol/status',self.on_status,20)
 def on_map(self,m):
  actual=np.asarray(m.data,dtype=np.int16).reshape(m.info.height,m.info.width)
  self.loaded={'frame':m.header.frame_id,'size':[m.info.width,m.info.height],'resolution':m.info.resolution,
    'origin':[m.info.origin.position.x,m.info.origin.position.y],
    'exact_cells_match':actual.shape==self.expected.shape and np.array_equal(actual,self.expected),
    'sha256':hashlib.sha256(actual.tobytes()).hexdigest()}
 def on_costmap(self,m):
  self.costmaps.append({'t':m.header.stamp.sec+m.header.stamp.nanosec/1e9,'frame':m.header.frame_id,
    'size':[m.info.width,m.info.height],'origin':[m.info.origin.position.x,m.info.origin.position.y],
    'resolution':m.info.resolution,'data':np.asarray(m.data,dtype=np.int16).reshape(m.info.height,m.info.width)})
 def on_path(self,m):
  points=[(p.pose.position.x,p.pose.position.y) for p in m.poses]
  lethal=[];unknown=[]
  for x,y in points:
   col=int(np.floor((x+2.1)/.02));row=int(np.floor((y+2.1)/.02))
   if not 0<=col<210 or not 0<=row<210:lethal.append([x,y]);continue
   v=int(self.expected[row,col])
   if v==100:lethal.append([x,y])
   if v<0:unknown.append([x,y])
  self.paths.append({'stamp':m.header.stamp.sec+m.header.stamp.nanosec/1e9,'frame':m.header.frame_id,
    'status':self.status,'points':points,'lethal_points':lethal,'unknown_points':unknown})
 def on_status(self,m):
  self.status=m.data
  if m.data.startswith('COMPLETE') and self.completed is None:self.completed=time.monotonic()
 def save(self):
  coverage={w['name']:any(any(((x-w['position'][0])**2+(y-w['position'][1])**2)**.5<.002 for x,y in path['points']) for path in self.paths)for w in WAYPOINTS}
  endpoints={w['name']:any(path['points'] and ((path['points'][-1][0]-w['position'][0])**2+(path['points'][-1][1]-w['position'][1])**2)**.5<.002 for path in self.paths)for w in WAYPOINTS if w['views'] or w['stop_for_light'] or w is WAYPOINTS[-1]}
  checks={'completed':self.completed is not None,'new_grid_exactly_loaded':bool(self.loaded) and self.loaded['exact_cells_match'],
    'field_frame':bool(self.loaded) and self.loaded['frame']=='field',
    'grid_metadata_match':bool(self.loaded) and abs(self.loaded['resolution']-.02)<1e-6 and all(abs(a-b)<1e-6 for a,b in zip(self.loaded['origin'],[-2.1,-2.1])),
    'all_waypoints_in_continuous_paths':all(coverage.values()),'semantic_stops_are_path_endpoints':all(endpoints.values()),
    'no_path_through_occupied_or_unknown':bool(self.paths) and all(not x['lethal_points'] and not x['unknown_points'] for x in self.paths),
    'live_global_costmap':len(self.costmaps)>=30}
  if self.costmaps:
   cv2.imwrite(str(self.out/'navigation-costmap.png'),np.flipud(np.where(self.costmaps[-1]['data']<0,205,254-self.costmaps[-1]['data']*2.54).astype(np.uint8)))
   same=[m for m in self.costmaps if m['data'].shape==self.expected.shape]
   if same:np.savez_compressed(self.out/'costmaps.npz',stamps=np.asarray([m['t'] for m in same]),grids=np.stack([m['data'] for m in same]))
  report={'passed':all(checks.values()) and self.error is None,'checks':checks,'error':self.error,'map_yaml':str(self.mapfile),
    'pgm_sha256':hashlib.sha256(self.mapfile.with_suffix('.pgm').read_bytes()).hexdigest(),'loaded':self.loaded,
    'paths':self.paths,'waypoint_coverage':coverage,'semantic_endpoints':endpoints,'costmap_frames':len(self.costmaps),
    'scope':'Nav2 input provenance and path feasibility; every waypoint lies on a published path within2mm, observation/light/final points are endpoints. Transit waypoints need not terminate a separate path. Actual movement, lane rules and post clearance independently checked by lap audit'}
  (self.out/'mapped-navigation-evidence.json').write_text(json.dumps(report,indent=2)+'\n')
  print(json.dumps({k:v for k,v in report.items() if k!='paths'}),flush=True);return report['passed']
def main():
 p=argparse.ArgumentParser();p.add_argument('--map',required=True);p.add_argument('--output',required=True);p.add_argument('--timeout',type=float,default=400);a=p.parse_args();out=Path(a.output);out.mkdir(exist_ok=True,parents=True)
 rclpy.init();n=Verify(Path(a.map),out);deadline=time.monotonic()+a.timeout
 try:
  while time.monotonic()<deadline:
   rclpy.spin_once(n,timeout_sec=.02)
   if n.completed is not None and time.monotonic()-n.completed>3:break
  else:n.error='Timeout: '+n.status
 except (Exception,KeyboardInterrupt) as e:n.error=repr(e)
 finally:passed=n.save();n.destroy_node();rclpy.try_shutdown()
 return 0 if passed else 1
if __name__=='__main__':raise SystemExit(main())
