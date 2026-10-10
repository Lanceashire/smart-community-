"""Real Nav2 planning in a separate DDS domain; no robot or Gazebo is launched."""
import argparse,json,os,sys,time,subprocess
from pathlib import Path
import cv2,numpy as np,rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile,DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import ComputePathToPose
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from run_simulation_refinement import stop
parser=argparse.ArgumentParser();parser.add_argument('--output',required=True);parser.add_argument('--domain',type=int,default=83);args=parser.parse_args()
os.environ['ROS_DOMAIN_ID']=str(args.domain)
OUT=Path(args.output).resolve();assert not (OUT/'result.json').exists(),'Retain previous evidence'
OUT.mkdir(parents=True,exist_ok=True)
pgm=cv2.imread(str(ROOT/'src/community_demo/maps/accepted-2026-10-06/navigation.pgm'),0)
yy,xx=np.mgrid[:210,:210];x=-2.1+(xx+.5)*.02;y=2.1-(yy+.5)*.02
mask=(x>.8)&(x<1.4)&(y>.95)&(y<1.15)
pgm[mask]=205;cv2.imwrite(str(OUT/'navigation.pgm'),pgm)
(OUT/'navigation.yaml').write_text((ROOT/'src/community_demo/maps/accepted-2026-10-06/navigation.yaml').read_text())
(OUT/'navigation-config.yaml').write_text((ROOT/'src/community_demo/config/navigation_mapped.yaml').read_text().replace('use_sim_time: true','use_sim_time: false'))
active=[];handles=[]
def spawn(cmd,label):
    f=(OUT/(label+'.log')).open('w');handles.append(f)
    q=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);active.append(q)
class Check(Node):
    def __init__(self):
        super().__init__('unknown_review_probe');self.costmap=None;self.map=None
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid,'/field_map',lambda m:setattr(self,'map',m),qos)
        self.create_subscription(OccupancyGrid,'/global_costmap/costmap',lambda m:setattr(self,'costmap',m),qos)
        self.client=ActionClient(self,ComputePathToPose,'/compute_path_to_pose')
report={'method':'Production Nav2 config, wall clock only, isolated ROS_DOMAIN_ID=83, synthetic unknown band; no Gazebo/actuator.'}
try:
    spawn(['ros2','run','tf2_ros','static_transform_publisher','--x','1.1','--y','.8','--frame-id','field','--child-frame-id','base_link'],'tf')
    spawn(['ros2','run','nav2_map_server','map_server','--ros-args','--params-file',str(OUT/'navigation-config.yaml'),'-p','yaml_filename:='+str(OUT/'navigation.yaml')],'map-server')
    spawn(['ros2','run','nav2_planner','planner_server','--ros-args','--params-file',str(OUT/'navigation-config.yaml')],'planner')
    spawn(['ros2','run','nav2_lifecycle_manager','lifecycle_manager','--ros-args','-r','__node:=review_nav_lifecycle','-p','autostart:=true','-p','node_names:=[map_server,planner_server]'],'lifecycle')
    rclpy.init();n=Check();deadline=time.monotonic()+35
    while time.monotonic()<deadline and (n.costmap is None or n.map is None):rclpy.spin_once(n,timeout_sec=.05)
    if n.map is None or n.costmap is None:raise RuntimeError('No active isolated Nav2 map/costmap')
    def cell(m,x,y):
        row=int((y-m.info.origin.position.y)//m.info.resolution);col=int((x-m.info.origin.position.x)//m.info.resolution)
        return m.data[row*m.info.width+col]
    report.update(raw_map_band_value=cell(n.map,1.1,1.05),costmap_band_value=cell(n.costmap,1.1,1.05))
    if not n.client.wait_for_server(timeout_sec=5):raise RuntimeError('Planner action unavailable')
    goal=ComputePathToPose.Goal();goal.use_start=True;goal.planner_id='GridBased'
    for p,py in ((goal.start,.8),(goal.goal,1.4)):
        p.header.frame_id='field';p.header.stamp=n.get_clock().now().to_msg();p.pose.position.x=1.1;p.pose.position.y=py;p.pose.orientation.w=1.
    future=n.client.send_goal_async(goal);rclpy.spin_until_future_complete(n,future,timeout_sec=15);handle=future.result()
    if handle is None or not handle.accepted:raise RuntimeError('Planning request not accepted')
    future=handle.get_result_async();rclpy.spin_until_future_complete(n,future,timeout_sec=15);response=future.result()
    if response is None:raise RuntimeError('No planner result')
    points=[(p.pose.position.x,p.pose.position.y) for p in response.result.path.poses]
    through=[p for p in points if cell(n.map,*p)<0]
    report.update(planner_status=response.status,path_points=len(points),points_inside_unknown=through,
                  unknown_discarded_and_planned_through=report['raw_map_band_value']==-1 and report['costmap_band_value']>=0 and response.status==4 and bool(through))
except Exception as exc:report['error']=repr(exc)
finally:
    for p in reversed(active):stop(p)
    for f in handles:f.close()
    if rclpy.ok():rclpy.shutdown()
    report['passed']=(report.get('raw_map_band_value')==-1 and report.get('costmap_band_value')==-1 and not report.get('points_inside_unknown',[]) and 'error' not in report)
    (OUT/'result.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))

raise SystemExit(0 if report['passed'] else 1)
