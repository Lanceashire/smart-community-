import argparse
#!/usr/bin/env python3
"""Regression: one physical target cannot count alternating identity labels twice."""
import json,math
from pathlib import Path
from community_demo.person_evidence import PersonEvidence
from community_demo.motion_profile import path_velocity

tracker=PersonEvidence();confirmed=[]
for frame,name in enumerate(['resident_01']*3+['visitor_01']*8,1):
    result=tracker.update([{'name':name,'confidence':.8,'bbox':[600,400,630,500]}],frame,0.,1280)
    confirmed.extend(d['name'] for d in result if d['confirmed'])
assert confirmed and set(confirmed)=={'resident_01'}
assert result[0]['identity_resolved'], 'Resolved track must not become a new pending person after a class flip'
tracker=PersonEvidence()
for frame in range(1,7):
    pan=0. if frame<4 else .08;f=1280/(2*math.tan(.8));cx=640-f*math.tan(math.atan(25/f)-pan)
    out=tracker.update([{'name':'resident_01','confidence':.85,'bbox':[cx-15,400,cx+15,500]}],frame,pan,1280)
assert tracker.next_id==2 and out[0]['confirmed']
out=tracker.update([{'name':'resident_01','confidence':.85,'bbox':[600,400,630,500]},
                    {'name':'visitor_01','confidence':.7,'bbox':[600,400,630,500]}],7,0.,1280)
assert len(out)==1
straight=[(i*.02,0.) for i in range(30)]
corner=[(i*.02,0.) for i in range(10)]+[(.18,i*.02) for i in range(1,15)]
assert path_velocity(straight,0,0.,1.,.22)[0]==.22
assert path_velocity(corner,6,0.,.4,.22)[0]<.22
assert path_velocity(straight,0,10.,1.,.22)[0]<=.065
report={'passed':True,'identity_flip_does_not_double_count':True,'pan_compensated_track':True,
        'overlapping_identity_boxes_suppressed':True,'bend_slowdown_before_turn':True,
        'scope':'Actual evidence association and motion profile functions, no ROS answers'}
parser=argparse.ArgumentParser();parser.add_argument('--output',default='validation/four-fixes-2026-10-04/association-regression.json');a=parser.parse_args()
out=Path(__file__).resolve().parents[1]/a.output
out.parent.mkdir(parents=True,exist_ok=True)
out.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
