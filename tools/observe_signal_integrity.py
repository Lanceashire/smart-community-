#!/usr/bin/env python3
"""Independent actual model-pose observation versus published signal phase."""
import argparse,json,signal,threading,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True);parser.add_argument('--duration',type=float,default=3600);a=parser.parse_args()
    rclpy.init();n=Node('independent_signal_integrity');gazebo=GzNode();lock=threading.Lock()
    observed={'phase':'unknown','received':0.,'changed':0.};advertised={'phase':'unknown','received':0.}
    diagnostics={};changes=[];bad=[];samples=0;start=time.monotonic();stop=False
    def poses(message):
        active={}
        for p in message.pose:
            if p.name.startswith(('light_top_','light_bottom_')) and p.name.endswith('_on') and p.position.z>.1:
                group='top' if p.name.startswith('light_top_') else 'bottom'
                active.setdefault(group,[]).append(p.name.split('_')[-2])
        phase=(active['top'][0] if len(active.get('top',[]))==len(active.get('bottom',[]))==1
               and active['top']==active['bottom'] else 'unknown')
        now=time.monotonic()
        with lock:
            if phase!=observed['phase']:
                changes.append({'wall_elapsed_s':now-start,'phase':phase});observed['changed']=now
            observed.update(phase=phase,received=now)
    def published(m):advertised.update(phase=m.data,received=time.monotonic())
    def diagnostic(m):diagnostics.update(json.loads(m.data))
    gazebo.subscribe(Pose_V,'/world/community/pose/info',poses)
    n.create_subscription(String,'/simulation/traffic_light_state',published,20)
    n.create_subscription(String,'/simulation/traffic_light_diagnostics',diagnostic,20)
    try:
        while time.monotonic()-start<a.duration:
            rclpy.spin_once(n,timeout_sec=.05);now=time.monotonic()
            with lock:o=observed.copy()
            if (o['phase']!='unknown' and advertised['phase']!='unknown' and
                now-o['received']<.3 and now-advertised['received']<.3 and now-o['changed']>.3):
                samples+=1
                if o['phase']!=advertised['phase']:
                    bad.append({'elapsed_s':now-start,'actual':o['phase'],'published':advertised['phase']})
    except KeyboardInterrupt:pass
    finally:
        report={'passed':samples>100 and len(changes)>=12 and not bad,'samples':samples,
            'actual_model_pose_transitions':changes,'stable_phase_mismatches':bad,
            'transition_grace_wall_s':.3,'latest_diagnostics':diagnostics,
            'scope':'Independent bright-lens model poses on both gantries; no patrol state injection'}
        out=Path(a.output);out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps({k:v for k,v in report.items() if k not in ('actual_model_pose_transitions',)},ensure_ascii=False))
        n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
