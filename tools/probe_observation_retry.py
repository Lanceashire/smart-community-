#!/usr/bin/env python3
"""Pause real perception at a plate view; check retry and recovery without fake labels."""
import argparse,json,os,signal,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from audit_patrol_cycles import process_for

def main():
    p=argparse.ArgumentParser();p.add_argument('--lap',type=int,default=1);p.add_argument('--output',default='validation/patrol-optimization-2026-10-02/retry-probe.json');a=p.parse_args()
    rclpy.init();n=Node('observation_retry_probe');state={};outcomes=[];pid=None;paused_at=None;resumed=False;saw_retry=False
    def result(m):
        nonlocal state
        state=json.loads(m.data)
    def outcome(m):outcomes.append(json.loads(m.data))
    def status(m):
        nonlocal saw_retry
        if m.data=='RETRY_OBSERVE plate_1':saw_retry=True
    n.create_subscription(String,'/perception/results',result,qos_profile_sensor_data)
    n.create_subscription(String,'/inspection/outcome',outcome,20)
    n.create_subscription(String,'/patrol/status',status,20)
    begin=time.monotonic()
    try:
        while time.monotonic()-begin<350:
            rclpy.spin_once(n,timeout_sec=.05)
            if paused_at is None and state.get('lap_id')==a.lap and state.get('view')=='plate_1':
                pid=process_for('perception');paused_at=time.monotonic();os.kill(pid,signal.SIGSTOP);print('RETRY_PROBE_PAUSED',pid,flush=True)
            if pid and time.monotonic()-paused_at>=7:
                os.kill(pid,signal.SIGCONT);pid=None;resumed=True;print('RETRY_PROBE_RESUMED',flush=True)
            target=next((x for x in outcomes if x.get('lap_id')==a.lap and x.get('view')=='plate_1'),None)
            if target:
                report={'passed':resumed and saw_retry and target['confirmed'] and target['elapsed_s']>5.5,'perception_pause_seconds':7,'saw_retry_status':saw_retry,'outcome':target,'method':'SIGSTOP/SIGCONT of actual perception at stationary observation; no substituted labels or robot poses'}
                out=Path(__file__).resolve().parents[1]/a.output;out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');print('RETRY_PROBE_RESULT',report['passed'],flush=True);return 0 if report['passed'] else 1
        raise RuntimeError('Observation retry probe timed out')
    finally:
        if pid:os.kill(pid,signal.SIGCONT)
        n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':raise SystemExit(main())
