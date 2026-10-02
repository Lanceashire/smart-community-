#!/usr/bin/env python3
"""Coverage probe: hold a stopped controller until red, resume, observe its own stop logic."""
import argparse,json,os,signal,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from std_msgs.msg import String
from audit_patrol_cycles import process_for

def main():
    p=argparse.ArgumentParser();p.add_argument('--lap',type=int,default=3);p.add_argument('--timeout',type=float,default=650);p.add_argument('--output',default='validation/patrol-cycle-review/lower-red-probe.json');a=p.parse_args()
    rclpy.init();n=Node('lower_red_coverage_probe');state={'lap':None,'phase':'unknown','status':''};events=[];held_pid=None;resumed_at=None;started=None
    n.create_subscription(String,'/patrol/lap',lambda m:state.update(lap=json.loads(m.data)['lap_id']),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    n.create_subscription(String,'/simulation/traffic_light_state',lambda m:state.update(phase=m.data),20)
    n.create_subscription(String,'/patrol/status',lambda m:state.update(status=m.data),20)
    begin=time.monotonic()
    try:
        while time.monotonic()-begin<a.timeout:
            rclpy.spin_once(n,timeout_sec=.05);now=time.monotonic()
            if state['lap']!=a.lap:continue
            if started is None and state['status'].startswith('STOP lower_stop'):
                if state['phase']=='red':
                    started=now;resumed_at=now;events.append({'action':'natural_red_arrival','state':state.copy()})
                else:
                    held_pid=process_for('patrol');os.kill(held_pid,signal.SIGSTOP);started=now;events.append({'action':'hold_stationary_controller','pid':held_pid,'state':state.copy()});print('LOWER_RED_HOLD',held_pid,flush=True)
            if held_pid and (state['phase']=='red' or now-started>30):
                os.kill(held_pid,signal.SIGCONT);events.append({'action':'resume_controller','hold_seconds':now-started,'state':state.copy()});held_pid=None;resumed_at=now;print('LOWER_RED_RESUMED',state['phase'],flush=True)
            if resumed_at is not None:
                if not events or events[-1].get('status')!=state['status'] or events[-1].get('phase')!=state['phase']:
                    events.append({'action':'after_resume','elapsed':now-resumed_at,**state})
                if state['status'].startswith('REACHED lower_stop'):break
        passed=resumed_at is not None and any(e.get('action')=='after_resume' and e.get('phase')=='red' and e.get('status')=='STOP lower_stop: light=red' for e in events) and state['status'].startswith('REACHED lower_stop') and state['phase']=='green'
        out=Path(__file__).resolve().parents[1]/a.output;out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps({'passed':passed,'lap_id':a.lap,'purpose':'Deliberately create a lower-stop red arrival; excluded from normal autonomous laps','events':events},indent=2)+'\n');print('LOWER_RED_RESULT',passed,flush=True)
        return 0 if passed else 1
    finally:
        if held_pid:
            try:os.kill(held_pid,signal.SIGCONT)
            except ProcessLookupError:pass
        n.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':raise SystemExit(main())
