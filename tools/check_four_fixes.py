import argparse
#!/usr/bin/env python3
"""Physical projection and phase safety counterexamples for the four fixes."""
import json
import math
from pathlib import Path
from community_demo.person_evidence import PersonEvidence
from community_demo.signal_gate import VisualPhaseClock, FreshGreenGate

checks={}
clock=VisualPhaseClock()
clock.observe('red',9.6);clock.observe('red',9.8);clock.observe('green',10.6)
g=FreshGreenGate(clock)
for i in range(5):g.observe('green',42.+i*.2)
checks['already_green_new_junction_releases_without_new_red']=g.ready
checks['remaining_lower_does_not_exceed_true_remaining']=g.remaining<=53.-42.8
for i in range(5):g.observe('green',51.+i*.2)
checks['late_green_insufficient_whole_body_budget_stops']=not g.ready
g.observe('green',55.)
checks['message_gap_does_not_reuse_green_streak']=not g.ready
cold=FreshGreenGate(VisualPhaseClock())
for i in range(10):cold.observe('green',i*.2)
checks['unobserved_cold_start_phase_not_invented']=not cold.ready
clock.observe('yellow',24.8);clock.observe('red',28.2)
checks['unobserved_long_gap_not_claimed_as_an_edge']=clock.edges==1
checks['expired_anchor_cannot_release']=clock.remaining_lower(1000.) is None
clock.observe('green',100.)
checks['fresh_matching_visual_phase_revalidates_clock']=clock.remaining_lower(100.) is not None

tracker=PersonEvidence();f=1280/(2*math.tan(.8));bearing=.15
ids=[]
for frame in range(1,13):
    pan=0. if frame<7 else .08
    # Independent pinhole equation: camera-local y points left, image u right.
    u=640-f*math.tan(bearing-pan)
    result=tracker.update([{'name':'resident_01','confidence':.85,'bbox':[u-14,400,u+14,500]}],frame,pan,1280)
    ids.append(result[0]['track_id'])
checks['physical_yaw_sweep_keeps_one_track']=len(set(ids))==1 and result[0]['confirmed']
for frame in range(13,21):
    result=tracker.update([{'name':'visitor_01','confidence':.85,'bbox':[u-14,400,u+14,500]}],frame,.08,1280)
checks['resolved_identity_flip_still_cannot_double_count']=not result[0]['confirmed'] and result[0]['identity_resolved']
report={'passed':all(checks.values()),'checks':checks,
        'scope':'Phase counterexamples and independent physical projection; full Gazebo acceptance separate'}
parser=argparse.ArgumentParser();parser.add_argument('--output',default='validation/four-fixes-2026-10-04/control-regression.json');a=parser.parse_args()
out=Path(__file__).resolve().parents[1]/a.output
out.parent.mkdir(parents=True,exist_ok=True)
out.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report));raise SystemExit(0 if report['passed'] else 1)
