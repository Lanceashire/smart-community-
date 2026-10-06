import argparse
#!/usr/bin/env python3
"""Exercise actual patrol callbacks with camera-derived phase reuse."""
import json
from pathlib import Path
from types import SimpleNamespace,MethodType
from rclpy.time import Time
from std_msgs.msg import String
from community_demo.patrol import Patrol
from community_demo.signal_gate import FreshGreenGate,VisualPhaseClock

clock=[0.]
phase=VisualPhaseClock()
node=SimpleNamespace(index=0,completed=False,signal_context=None,
                     phase_clock=phase,signal_gate=FreshGreenGate(phase),
                     get_clock=lambda:SimpleNamespace(now=lambda:Time(seconds=clock[0])))
node.update_signal_context=MethodType(Patrol.update_signal_context,node)
def observe(t,colour):
    clock[0]=t
    Patrol.on_light(node,String(data=colour))
observe(9.6,'red');observe(9.8,'red')
for i in range(5):observe(10.6+i*.2,'green')
assert node.signal_gate.ready
node.index=1;node.update_signal_context()
assert node.signal_gate.ready
node.index=2;node.update_signal_context()
assert not node.signal_gate.ready
node.index=9
for i in range(5):observe(42.+i*.2,'green')
assert node.signal_gate.ready
observe(43.8,'green')
assert not node.signal_gate.ready
for i in range(5):observe(51.+i*.2,'green')
assert not node.signal_gate.ready
observe(52.,'unknown')
for i in range(5):observe(52.2+i*.2,'green')
assert not node.signal_gate.ready
node.completed=True;node.update_signal_context()
assert not node.signal_gate.ready
commands=[];statuses=[]
node.tilt=0.;node.fault=None;node.distance_travelled=12.5
node.pose=None;node.pose_update=None;node.pose_source_time=None
node.motion_probe=object()
node.publish_command=lambda:commands.append((0.,0.))
node.log_status=statuses.append
Patrol.tick(node)
assert commands==[(0.,0.)] and statuses[-1].startswith('COMPLETE') and node.motion_probe is None
node.fault='FAULT acceptance counterexample'
Patrol.tick(node)
assert commands[-1]==(0.,0.) and statuses[-1]==node.fault
report={'passed':True,'actual_callback_approach_to_stop_retains_evidence':True,
        'new_junction_reuses_phase_but_requires_fresh_green_streak':True,
        'vision_gap_invalidates_local_green_streak':True,'late_green_stops':True,
        'unknown_cannot_invent_a_new_green':True,'completion_resets_local_gate':True,
        'completed_lap_stays_stopped_and_latched_with_all_sensors_missing':True,
        'completion_does_not_mask_a_latched_fault':True,
        'scope':'Actual patrol callbacks; camera colours only, no simulator topic'}
parser=argparse.ArgumentParser();parser.add_argument('--output',default='validation/four-fixes-2026-10-04/callback-regression.json');a=parser.parse_args()
out=Path(__file__).resolve().parents[1]/a.output
out.parent.mkdir(parents=True,exist_ok=True)
out.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
