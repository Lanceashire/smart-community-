#!/usr/bin/env python3
"""Move a physical obstacle into and out of the permitted route; truth is audit only."""
import argparse,json,math,subprocess,time
from pathlib import Path
import rclpy
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from community_demo.road_rules import HALF_LENGTH,HALF_WIDTH

NAME='dynamic_acceptance_obstacle'
def service(name,kind,request):
    p=subprocess.run(['gz','service','-s','/world/community/'+name,'--reqtype',kind,
                     '--reptype','gz.msgs.Boolean','--timeout','1500','--req',request],
                    capture_output=True,text=True,timeout=3)
    if p.returncode or 'data: true' not in p.stdout:raise RuntimeError(p.stdout+p.stderr)

def clearance(pose,bx,by):
    x,y,yaw=pose;c,s=math.cos(yaw),math.sin(yaw);dx=bx-x;dy=by-y
    # SAT separation: positive means separated; negative means intersection.
    axes=[(1.,0.),(0.,1.),(c,s),(-s,c)];gaps=[]
    for ax,ay in axes:
        radius=HALF_LENGTH*abs(ax*c+ay*s)+HALF_WIDTH*abs(-ax*s+ay*c)+.07*(abs(ax)+abs(ay))
        gaps.append(abs(dx*ax+dy*ay)-radius)
    return max(gaps)

class Probe(Node):
    def __init__(self):
        super().__init__('dynamic_interference_audit');self.pose=None;self.status='';self.events=[];self.samples=[]
        self.obstacle=None;self.command=(0.,0.);self.sim=None
        self.observed_obstacle=None;self.obstacle_pose_updates=0;self.gz=GzNode()
        self.gz.subscribe(Pose_V,'/world/community/pose/info',self.obstacle_pose)
        self.create_subscription(String,'/patrol/status',self.status_cb,50)
        self.create_subscription(Odometry,'/model/patrol_bot/ground_truth',self.truth,qos_profile_sensor_data)
        from geometry_msgs.msg import Twist
        self.create_subscription(Twist,'/model/patrol_bot/cmd_vel',lambda m:setattr(self,'command',(m.linear.x,m.angular.z)),20)
    def status_cb(self,m):
        if m.data!=self.status:self.events.append({'sim':self.sim,'status':m.data})
        self.status=m.data
    def obstacle_pose(self,message):
        for p in message.pose:
            if p.name==NAME:
                self.observed_obstacle=(p.position.x,p.position.y)
                self.obstacle_pose_updates+=1
    def truth(self,m):
        p=m.pose.pose.position;q=m.pose.pose.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        self.sim=m.header.stamp.sec+m.header.stamp.nanosec/1e9
        self.pose=(p.x,p.y,yaw,math.hypot(m.twist.twist.linear.x,m.twist.twist.linear.y))
        obstacle=self.observed_obstacle
        if obstacle:
            self.samples.append({'sim':self.sim,'pose':list(self.pose),'obstacle':obstacle,
                                 'clearance_m':clearance(self.pose[:3],*obstacle),'command':self.command,'status':self.status})

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);a=p.parse_args()
    out=Path(a.output);out.parent.mkdir(parents=True,exist_ok=True)
    rclpy.init();n=Probe();created=False;stage='waiting';started=None;blocked=None;depart=None;clear_at=None
    report={'passed':False,'method':'Collidable 14cm box moves laterally into the right road at 0.4m/s, stays until the robot stops, then leaves. Only obstacle pose services; robot control receives actual sensor data.',
            'capability':'Stop, wait, confirm clear scans, replan the permitted route and resume; active bypass is not claimed'}
    deadline=time.monotonic()+360
    try:
        while time.monotonic()<deadline:
            rclpy.spin_once(n,timeout_sec=.02)
            if n.pose is None:continue
            if stage=='waiting' and n.status.startswith('GO right_north') and -.35<n.pose[1]<-.15 and n.pose[3]>.07:
                by=n.pose[1]+.9;bx=1.65
                sdf=f'<sdf version="1.9"><model name="{NAME}"><static>true</static><pose>{bx} {by} .18 0 0 0</pose><link name="body"><collision name="collision"><geometry><box><size>.14 .14 .36</size></box></geometry></collision><visual name="visual"><geometry><box><size>.14 .14 .36</size></box></geometry><material><diffuse>.9 .2 .1 1</diffuse></material></visual></link></model></sdf>'
                service('create','gz.msgs.EntityFactory','sdf: '+json.dumps(sdf));created=True
                started=n.sim;last_move=-1e9;stage='enter';report.update(start_sim=started,obstacle_y=by,initial_robot_speed_m_s=n.pose[3])
                print('DYNAMIC_OBSTACLE_ENTER',flush=True)
            if stage=='enter':
                bx=max(1.1,1.65-.4*(n.sim-started))
                if n.sim-last_move>=.1:
                    service('set_pose','gz.msgs.Pose',f'name: "{NAME}" position {{ x: {bx} y: {by} z: .18 }} orientation {{ w: 1 }}')
                    n.obstacle=(bx,by);last_move=n.sim
                if bx<=1.10001:stage='hold';entered=n.sim
            if stage=='hold':
                if n.status.startswith('BLOCKED') and n.pose[3]<.015 and blocked is None:
                    blocked=n.sim;report.update(blocked_sim=blocked,stopped_robot_pose=list(n.pose),stop_clearance_m=clearance(n.pose[:3],bx,by))
                    print('DYNAMIC_OBSTACLE_STOPPED',flush=True)
                if blocked is not None and n.sim-blocked>=3.:
                    depart=n.sim;stage='leave'
            if stage=='leave':
                bx=max(.55,1.1-.4*(n.sim-depart))
                if n.sim-last_move>=.1:
                    service('set_pose','gz.msgs.Pose',f'name: "{NAME}" position {{ x: {bx} y: {by} z: .18 }} orientation {{ w: 1 }}')
                    n.obstacle=(bx,by);last_move=n.sim
                if bx<=.55001:stage='clear';clear_at=n.sim;report['clear_sim']=clear_at
            if stage=='clear' and n.pose[1]>by+.4:
                late=[s for s in n.samples if blocked is not None and blocked+.5<=s['sim']<=depart]
                recovery=any('RECOVER_OBSTACLE replan permitted route' in e['status'] for e in n.events)
                checks={'moving_at_start':report['initial_robot_speed_m_s']>.07,'obstacle_caused_stop':blocked is not None,
                        'actual_obstacle_pose_measured':n.obstacle_pose_updates>10,
                        'no_body_overlap':bool(n.samples) and min(s['clearance_m'] for s in n.samples)>0,
                        'stationary_while_blocked':bool(late) and max(s['pose'][3] for s in late)<.015 and all(max(map(abs,s['command']))<.001 for s in late),
                        'clearance_confirmed_and_replanned':recovery,'resumed_past_obstacle':n.pose[1]>by+.4}
                report.update(passed=all(checks.values()),checks=checks,minimum_body_clearance_m=min(s['clearance_m'] for s in n.samples),
                              hold_speed_max_m_s=max((s['pose'][3] for s in late),default=None),resume_sim=n.sim)
                break
        else:report['error']='Timed out in stage '+stage+'; '+n.status
    except Exception as exc:report['error']=str(exc)
    finally:
        if created:
            try:service('remove','gz.msgs.Entity',f'name: "{NAME}", type: MODEL')
            except Exception as exc:report['cleanup_error']=str(exc);report['passed']=False
        report.update(events=n.events,samples=n.samples,actual_obstacle_pose_updates=n.obstacle_pose_updates)
        out.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items() if k not in ('samples','events')}),flush=True)
        n.destroy_node();rclpy.try_shutdown()
    return 0 if report['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
