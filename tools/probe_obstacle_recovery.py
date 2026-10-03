#!/usr/bin/env python3
"""Place and remove one physical obstacle; observe stop/replan without command injection."""
import argparse,json,math,subprocess,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import String

NAME='performance_recovery_obstacle'
class Probe(Node):
    def __init__(self):
        super().__init__('physical_obstacle_recovery_probe');self.status='';self.pose=None;self.events=[]
        self.create_subscription(String,'/patrol/status',self.status_cb,50)
        self.create_subscription(Odometry,'/model/patrol_bot/ground_truth',self.truth,qos_profile_sensor_data)
    def status_cb(self,m):
        if m.data!=self.status:self.events.append({'wall':time.monotonic(),'status':m.data})
        self.status=m.data
    def truth(self,m):
        p=m.pose.pose.position;self.pose=(p.x,p.y,math.hypot(m.twist.twist.linear.x,m.twist.twist.linear.y))

def service(name,reqtype,req):
    result=subprocess.run(['gz','service','-s','/world/community/'+name,'--reqtype',reqtype,'--reptype','gz.msgs.Boolean','--timeout','5000','--req',req],capture_output=True,text=True,timeout=9)
    if result.returncode or 'data: true' not in result.stdout:raise RuntimeError(result.stdout+result.stderr)

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);a=p.parse_args();out=Path(a.output);out.parent.mkdir(parents=True,exist_ok=True)
    rclpy.init();n=Probe();created=False;blocked=None;removed=None;speeds=[];deadline=time.monotonic()+420
    report={'passed':False,'method':'Physical Gazebo box added then removed; evaluator reads truth; no robot teleport, velocity command or perception answer injection'}
    try:
        while time.monotonic()<deadline:
            rclpy.spin_once(n,timeout_sec=.05);now=time.monotonic()
            if not created and n.pose and n.status.startswith('GO right_north') and -.3<n.pose[1]<0:
                sdf=f'<sdf version="1.9"><model name="{NAME}"><static>true</static><pose>1.1 0.75 0.12 0 0 0</pose><link name="box"><collision name="collision"><geometry><box><size>0.14 0.14 0.24</size></box></geometry></collision><visual name="visual"><geometry><box><size>0.14 0.14 0.24</size></box></geometry><material><diffuse>0.9 0.2 0.1 1</diffuse></material></visual></link></model></sdf>'
                service('create','gz.msgs.EntityFactory','sdf: '+json.dumps(sdf));created=True;report['spawn_pose']=[1.1,.75,.12];print('OBSTACLE_CREATED',flush=True)
            if created and removed is None and n.status.startswith('BLOCKED'):
                if blocked is None:blocked=now;report['stopped_robot_pose']=n.pose[:2]
                if now-blocked>1 and n.pose:speeds.append(n.pose[2])
                if now-blocked>=3:
                    service('remove','gz.msgs.Entity',f'name: "{NAME}", type: MODEL');removed=now;print('OBSTACLE_REMOVED',flush=True)
            if removed and n.pose and n.pose[1]>1.0:
                recovery=any(e['status'].startswith('RECOVER_OBSTACLE replan') for e in n.events)
                report.update(passed=bool(speeds) and max(speeds)<.015 and recovery,settled_max_speed_m_s=max(speeds,default=None),clearance_to_box_at_stop_m=.68-report['stopped_robot_pose'][1]-.11,replanned=recovery,resumed_y=n.pose[1],recovery_wall_s=now-removed)
                break
        else:report['error']='Timed out waiting for obstacle stop/recovery'
    except Exception as exc:report['error']=str(exc)
    finally:
        if created and removed is None:
            try:service('remove','gz.msgs.Entity',f'name: "{NAME}", type: MODEL')
            except Exception as exc:report['cleanup_error']=str(exc)
        report['events']=[e for e in n.events if 'BLOCKED' in e['status'] or 'RECOVER' in e['status']]
        out.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report),flush=True);n.destroy_node();rclpy.try_shutdown()
    return 0 if report['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
