#!/usr/bin/env python3
"""Read-only autonomous SLAM capture, map export and chronological video."""
import argparse,hashlib,json,math,subprocess,time
from pathlib import Path
import cv2,numpy as np,rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid,Odometry
from sensor_msgs.msg import Image,LaserScan
from std_msgs.msg import String
from slam_toolbox.srv import SerializePoseGraph
from tf2_ros import Buffer,TransformListener
from rclpy.time import Time
from rclpy.parameter import Parameter
from community_demo.sensor_validation import valid_laser_scan,stamp_ns
from rclpy.signals import SignalHandlerOptions
from PIL import Image as PILImage,ImageDraw,ImageFont
from community_demo.mapping_grid import field_grid,navigation_grid,image,RESOLUTION,ORIGIN,free_ray_indices,fuse_registered_rays,update_registered_votes
ROOT=Path(__file__).resolve().parents[1]
FONT=ROOT/'src/community_demo/models/fonts/wqy-zenhei.ttc'
def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
class Capture(Node):
 def __init__(self,out):
  super().__init__('autonomous_mapping_evidence');self.set_parameters([Parameter('use_sim_time',value=True)]);self.out=out;self.map=None;self.stats=[];self.scans=0;self.scan_stamps=set();self.status='Waiting';self.pose=None;self.path=[];self.overview=None;self.completed=None;self.key=None
  self.tf=Buffer();self.listener=TransformListener(self.tf,self)
  self.create_subscription(OccupancyGrid,'/map',self.on_map,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
  self.create_subscription(LaserScan,'/scan',self.on_scan,qos_profile_sensor_data)
  self.create_subscription(Odometry,'/localization/pose',self.on_pose,qos_profile_sensor_data)
  self.create_subscription(String,'/patrol/status',self.on_status,20)
  self.create_subscription(String,'/patrol/lap',lambda m:setattr(self,'key',json.loads(m.data)),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
  self.create_subscription(Image,'/camera/image',self.on_overview,qos_profile_sensor_data)
  self.font=ImageFont.truetype(str(FONT),20);self.small=ImageFont.truetype(str(FONT),16)
  self.encoder=None;self.frames=0;self.start=None;self.trans=None;self.error=None;self.hit_counts=np.zeros((210,210),np.uint16);self.free_counts=np.zeros((210,210),np.uint16);self.pending_scans=[];self.registered_scans=0;self.last_registered_ns=None
 def on_scan(self,m):
  source=stamp_ns(m.header.stamp)
  if not valid_laser_scan(m) or source in self.scan_stamps:return
  self.scans+=1;self.scan_stamps.add(source);self.pending_scans.append(m);self.pending_scans=self.pending_scans[-20:]
 def register_hits(self):
  pending=[]
  for m in sorted(self.pending_scans,key=lambda m:stamp_ns(m.header.stamp)):
   source=stamp_ns(m.header.stamp)
   if self.last_registered_ns is not None and source<=self.last_registered_ns:continue
   if not 0<=self.get_clock().now().nanoseconds-source<=500_000_000:continue
   try:t=self.tf.lookup_transform('field','base_link',Time(nanoseconds=source)).transform
   except Exception:pending.append(m);continue
   a=yaw(t.rotation);r=np.asarray(m.ranges);i=np.flatnonzero(np.isfinite(r)&(r>=m.range_min)&(r<5.95))
   angles=a+m.angle_min+i*m.angle_increment
   x=t.translation.x+r[i]*np.cos(angles);y=t.translation.y+r[i]*np.sin(angles)
   col=np.floor((x+2.1)/.02).astype(int);row=np.floor((y+2.1)/.02).astype(int)
   valid=(col>=0)&(row>=0)&(col<210)&(row<210)
   indices=np.unique(row[valid]*210+col[valid])
   all_angles=a+m.angle_min+np.arange(len(r))*m.angle_increment
   clear=free_ray_indices((t.translation.x,t.translation.y),all_angles,r,m.range_min,m.range_max)
   update_registered_votes(self.hit_counts,self.free_counts,indices,clear)
   self.last_registered_ns=source
   self.registered_scans+=1
  self.pending_scans=pending
 def on_status(self,m):
  self.status=m.data
  if m.data.startswith('COMPLETE') and self.completed is None:self.completed=time.monotonic()
 def on_pose(self,m):
  self.pose=(m.pose.pose.position.x,m.pose.pose.position.y,yaw(m.pose.pose.orientation))
  if not self.path or math.hypot(self.pose[0]-self.path[-1][0],self.pose[1]-self.path[-1][1])>.01:self.path.append(self.pose)
 def on_overview(self,m):
  if m.encoding not in ('rgb8','bgr8') or len(m.data)!=m.step*m.height:return
  a=np.frombuffer(m.data,np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3)
  self.overview=cv2.cvtColor(a,cv2.COLOR_BGR2RGB) if m.encoding=='bgr8' else a.copy()
 def on_map(self,m):
  if m.header.frame_id!='slam_map' or not m.info.width or not m.info.height:return
  self.map=m;grid=np.asarray(m.data,dtype=np.int16)
  d={'stamp_s':m.header.stamp.sec+m.header.stamp.nanosec/1e9,'size':[m.info.width,m.info.height],
     'known_cells':int((grid>=0).sum()),'occupied_cells':int((grid>65).sum()),
     'sha256':hashlib.sha256(grid.tobytes()).hexdigest()}
  self.stats.append(d)
  if len(self.stats)==1:
   cv2.imwrite(str(self.out/'first-map.png'),image(grid.reshape(m.info.height,m.info.width)))
   print('FIRST_LIVE_SLAM_MAP',json.dumps(d),flush=True)
 def project(self):
  t=self.tf.lookup_transform('slam_map','field',Time()).transform
  self.trans=(t.translation.x,t.translation.y,yaw(t.rotation))
  grid=field_grid(self.map,self.trans)
  return fuse_registered_rays(grid,self.hit_counts,self.free_counts)
 def render(self):
  if self.map is None or self.overview is None:return
  try:grid=self.project()
  except Exception:return
  now=time.monotonic()
  if self.start is None:
   self.start=now
   self.encoder=subprocess.Popen(['ffmpeg','-hide_banner','-loglevel','error','-y','-f','rawvideo','-pix_fmt','rgb24','-s','1280x720','-r','2','-i','pipe:0','-an','-c:v','libx264','-preset','veryfast','-crf','18','-threads','2','-pix_fmt','yuv420p','-movflags','+faststart',str(self.out/'mapping.mp4')],stdin=subprocess.PIPE)
  expected=int((now-self.start)*2)+1
  if self.frames>=expected:return
  panel=PILImage.new('RGB',(1280,720),(14,22,36));draw=ImageDraw.Draw(panel)
  draw.text((16,8),'自主建图实录：机器人实际相机 + 实时 SLAM 地图',font=self.font,fill='white')
  panel.paste(PILImage.fromarray(self.overview).resize((640,640)),(0,45))
  pixels=cv2.cvtColor(image(grid),cv2.COLOR_GRAY2RGB)
  for x,y,a in self.path:
   u=int((x+2.1)/.02);v=209-int((y+2.1)/.02)
   if 0<=u<210 and 0<=v<210:cv2.circle(pixels,(u,v),1,(0,170,250),-1)
  if self.pose:
   u=int((self.pose[0]+2.1)/.02);v=209-int((self.pose[1]+2.1)/.02);cv2.circle(pixels,(u,v),3,(255,70,40),-1)
  panel.paste(PILImage.fromarray(pixels).resize((640,640),PILImage.Resampling.NEAREST),(640,45))
  draw.text((20,48),'机器人实际相机',font=self.small,fill='#70e5d0')
  draw.text((650,48),'激光地图：黑=障碍 白=已知空闲 灰=未知',font=self.small,fill='#003b80')
  s=self.stats[-1];draw.text((16,686),f'{s["stamp_s"]:.1f}s | 地图更新 {len(self.stats)} | 激光帧 {self.scans} | {self.status[:55]}',font=self.small,fill='white')
  raw=panel.tobytes()
  for _ in range(expected-self.frames):self.encoder.stdin.write(raw);self.frames+=1
 def save(self):
  checks={'completed_autonomous_route':self.completed is not None,'live_map_updates':len(self.stats)>=30,
          'distinct_live_grids':len({x['sha256'] for x in self.stats})>=20,'unique_laser_scans':len(self.scan_stamps)>=500,
          'map_frame_correct':self.map is not None and self.map.header.frame_id=='slam_map'}
  evidence={}
  if self.map is not None:
   raw=np.asarray(self.map.data,dtype=np.int16).reshape(self.map.info.height,self.map.info.width)
   cv2.imwrite(str(self.out/'slam.pgm'),image(raw));self.yaml('slam',self.map.info.resolution,(self.map.info.origin.position.x,self.map.info.origin.position.y,yaw(self.map.info.origin.orientation)))
   grid=self.project();cv2.imwrite(str(self.out/'slam-field.pgm'),image(grid));self.yaml('slam-field',RESOLUTION,(*ORIGIN,0))
   nav,evidence=navigation_grid(grid);evidence.update(registered_laser_frames=self.registered_scans,confirmed_hit_cells=int((self.hit_counts>=3).sum()),confirmed_free_ray_cells=int((self.free_counts>=3).sum()),inconsistent_hit_cells_cleared=int(((self.hit_counts>=3)&(grid==0)).sum()),source='SLAM OccupancyGrid, >=3-frame endpoints and unique-per-frame clear rays registered by camera/wheel/IMU source-time TF; clear requires >=3 frames and >=3 clear votes per hit; supplied lanes separately constrain travel; no SDF collision coordinates');cv2.imwrite(str(self.out/'navigation.pgm'),image(nav));self.yaml('navigation',RESOLUTION,(*ORIGIN,0))
   cv2.imwrite(str(self.out/'final-map.png'),image(grid))
   checks['known_road_coverage']=evidence['known_road_fraction']>=.95
   checks['measured_physical_obstacles']=int((grid>65).sum())>=20
   checks['known_map_growth']=self.stats[-1]['known_cells']>self.stats[0]['known_cells']
   np.savez_compressed(self.out/'occupancy.npz',slam=raw,field=grid,navigation=nav,laser_hit_counts=self.hit_counts,laser_free_counts=self.free_counts)
   client=self.create_client(SerializePoseGraph,'/slam_toolbox/serialize_map')
   checks['posegraph_saved']=False
   if client.wait_for_service(timeout_sec=3):
    req=SerializePoseGraph.Request();req.filename=str(self.out/'slam-graph');f=client.call_async(req);rclpy.spin_until_future_complete(self,f,timeout_sec=8)
    checks['posegraph_saved']=f.done() and f.result() is not None and f.result().result==0 and (self.out/'slam-graph.posegraph').exists() and (self.out/'slam-graph.data').exists()
  if self.encoder:
   self.encoder.stdin.close();self.encoder.wait(timeout=20)
  checks['continuous_map_video']=self.encoder is not None and self.encoder.returncode==0
  distance=sum(math.hypot(b[0]-a[0],b[1]-a[1]) for a,b in zip(self.path,self.path[1:]));checks['entire_route_travel']=distance>12
  hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in self.out.glob('*') if p.suffix in ('.pgm','.yaml','.posegraph','.data','.npz')}
  report={'passed':all(checks.values()) and self.error is None,'checks':checks,'error':self.error,'run':self.key,'map_statistics':self.stats,
          'unique_scan_frames':len(self.scan_stamps),'estimated_travel_m':distance,'field_to_slam_tf':self.trans,'planning_grid':evidence,'map_hashes':hashes,
          'source':'Fresh SLAM Toolbox mapping process, actual /scan and wheel/camera/IMU localization; no previously saved SLAM grid loaded',
          'autonomy':'Known competition route drives the robot; human teleoperation and ground truth are absent from mapping control',
          'video_frames':self.frames,'video_fps':2,'scope':'Known-field autonomous route mapping, not unknown-environment frontier exploration; supplied lane geometry is disclosed separately'}
  (self.out/'mapping-evidence.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
  print(json.dumps({k:v for k,v in report.items() if k not in ('map_statistics','map_hashes')},ensure_ascii=False),flush=True)
  return report['passed']
 def yaml(self,name,res,origin):
  (self.out/(name+'.yaml')).write_text(f'image: {name}.pgm\nmode: trinary\nresolution: {res}\norigin: [{origin[0]}, {origin[1]}, {origin[2]}]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
def main():
 p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--timeout',type=float,default=400);a=p.parse_args();out=ROOT/a.output;out.mkdir(parents=True,exist_ok=True)
 if (out/'mapping-evidence.json').exists():raise RuntimeError('Preserve previous mapping evidence')
 rclpy.init(signal_handler_options=SignalHandlerOptions.NO);n=Capture(out);deadline=time.monotonic()+a.timeout
 try:
  while time.monotonic()<deadline:
   rclpy.spin_once(n,timeout_sec=.01);n.register_hits();n.render()
   if n.completed is not None and time.monotonic()-n.completed>3:break
  else:n.error='Mapping timeout: '+n.status
 except (Exception,KeyboardInterrupt) as e:n.error=repr(e)
 finally:passed=n.save();n.destroy_node();rclpy.try_shutdown()
 return 0 if passed else 1
if __name__=='__main__':raise SystemExit(main())
