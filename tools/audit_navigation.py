#!/usr/bin/env python3
"""Independent evaluator only: compare estimated pose to simulator truth."""
import argparse,json,math,time
from pathlib import Path
import rclpy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from community_demo.road_rules import footprint,in_road
from community_demo.patrol import yaw_from_quaternion,wrap
p=argparse.ArgumentParser();p.add_argument('--timeout',type=float,default=300);p.add_argument('--output',default='validation/navigation-audit.json');a=p.parse_args()
rclpy.init();n=rclpy.create_node('independent_navigation_audit');state={'estimated':None,'phase':'unknown','status':''};records=[];crossings=[];last=None
n.create_subscription(Odometry,'/localization/pose',lambda m:state.update(estimated=(m.pose.pose.position.x,m.pose.pose.position.y,yaw_from_quaternion(m.pose.pose.orientation))),10)
n.create_subscription(String,'/simulation/traffic_light_state',lambda m:state.update(phase=m.data),10)
n.create_subscription(String,'/patrol/status',lambda m:state.update(status=m.data),10)
def truth(m):
    global last
    x,y,yaw=m.pose.pose.position.x,m.pose.pose.position.y,yaw_from_quaternion(m.pose.pose.orientation)
    points=footprint(x,y,yaw);outside=sum(not in_road(u,v,.004) for u,v in points)
    error=None if state['estimated'] is None else [math.hypot(x-state['estimated'][0],y-state['estimated'][1]),abs(wrap(yaw-state['estimated'][2]))]
    t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
    # Signed body-front distance before each stop line along prescribed direction.
    front=[min(u for u,v in points)-.30,min(v for u,v in points)+.51]
    if last:
        for i in range(2):
            near=(abs(y-1.7)<.3 if i==0 else abs(x+.1)<.3)
            if near and last[i]>=0 and front[i]<0:crossings.append({'gate':i+1,'time':t,'phase':state['phase'],'legal':state['phase']=='green'})
    last=front;records.append({'t':t,'truth':[x,y,yaw],'estimate':state['estimated'],'error':error,'outside_points':outside})
n.create_subscription(Odometry,'/model/patrol_bot/ground_truth',truth,10)
end=time.monotonic()+a.timeout
while time.monotonic()<end:
    rclpy.spin_once(n,timeout_sec=.1)
    if state['status'].startswith(('COMPLETE','FAULT')):break
errors=[r['error'] for r in records if r['error']]
report={'status':state['status'],'samples':len(records),'max_position_error_m':max((e[0] for e in errors),default=None),'max_yaw_error_deg':math.degrees(max((e[1] for e in errors),default=0)),'footprint_violation_samples':sum(r['outside_points']>0 for r in records),'stop_line_crossings':crossings,'records':records}
Path(a.output).parent.mkdir(parents=True,exist_ok=True);Path(a.output).write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k!='records'},indent=2));n.destroy_node();rclpy.shutdown()
