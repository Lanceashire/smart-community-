#!/usr/bin/env python3
"""Pause real perception while driving and measure watchdog stop/recovery."""
import argparse,json,os,signal,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from audit_patrol_cycles import process_for

def main():
    p=argparse.ArgumentParser();p.add_argument('--lap',type=int,required=True);p.add_argument('--output',required=True);a=p.parse_args()
    rclpy.init();n=Node('perception_motion_probe');state={};status='';pose=None;command=(0.,0.);pid=None;started=None;resumed=None;samples=[];saw_wait=False
    def result(m):
        nonlocal state
        state=json.loads(m.data)
    def on_status(m):
        nonlocal status,saw_wait
        status=m.data
        if started is not None and status=='WAIT_FRESH_PERCEPTION':saw_wait=True
    def truth(m):
        nonlocal pose
        pose=(m.pose.pose.position.x,m.pose.pose.position.y,abs(m.twist.twist.linear.x))
        if started is not None and resumed is None:samples.append({'elapsed_wall_s':time.monotonic()-started,'pose':pose[:2],'speed_m_s':pose[2],'command':command,'status':status})
    def cmd(m):
        nonlocal command
        command=(m.linear.x,m.angular.z)
    n.create_subscription(String,'/perception/results',result,qos_profile_sensor_data)
    n.create_subscription(String,'/patrol/status',on_status,50)
    n.create_subscription(Odometry,'/model/patrol_bot/ground_truth',truth,qos_profile_sensor_data)
    n.create_subscription(Twist,'/model/patrol_bot/cmd_vel',cmd,20)
    report={'passed':False,'lap_id':a.lap,'method':'Actual SIGSTOP/SIGCONT of perception on straight moving segment. Physical truth and commands are read only.'};end=time.monotonic()+420
    try:
        while time.monotonic()<end:
            rclpy.spin_once(n,timeout_sec=.05);now=time.monotonic()
            if started is None and state.get('lap_id')==a.lap and status.startswith('GO top_west') and pose and -.98<pose[0]<-.8 and pose[2]>.07:
                pid=process_for('perception');started=now;os.kill(pid,signal.SIGSTOP);print('MOTION_PERCEPTION_PAUSED',flush=True)
            if pid is not None and now-started>=2.5:
                os.kill(pid,signal.SIGCONT);pid=None;resumed=now;print('MOTION_PERCEPTION_RESUMED',flush=True)
            if resumed and pose and pose[2]>.07 and status.startswith('GO top_west'):
                late=[s for s in samples if s['elapsed_wall_s']>=1.5]
                zero=bool(late) and all(max(map(abs,s['command']))<.001 for s in late)
                speed=max((s['speed_m_s'] for s in late),default=999.)
                report.update(passed=zero and speed<.015 and saw_wait,saw_wait_fresh_perception=saw_wait,settled_zero_command=zero,settled_max_speed_m_s=speed,pause_wall_s=resumed-started,recovery_wall_s=now-resumed,samples=samples);break
        else:report['error']='Timed out waiting for actual moving perception stop/recovery'
    except Exception as e:report['error']=str(e)
    finally:
        if pid is not None:os.kill(pid,signal.SIGCONT)
        out=Path(__file__).resolve().parents[1]/a.output;out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(report,indent=2)+'\n');print('MOTION_PERCEPTION_RESULT',report['passed'],flush=True);n.destroy_node();rclpy.try_shutdown()
    return 0 if report['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
