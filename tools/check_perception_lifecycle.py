#!/usr/bin/env python3
"""Replay delayed status and completed-report recovery without publishing ROS commands."""
import json
from collections import deque
from pathlib import Path
from types import SimpleNamespace
from std_msgs.msg import String
from community_demo.perception import Perception

root=Path(__file__).resolve().parents[1]
saved=json.loads((root/'validation/patrol-cycle-review/lap-002.json').read_text())['perception']
node=SimpleNamespace(run_id='old',lap_id=8,people={'old':{}},plates={'old':{}},events=[{}],votes={'old':[]},light_history=deque(['red']),last_light_event='red',last_ocr=10.,view='plate_1',completed=True,status='COMPLETE old',frame_id=1,previous_report={},out=root/'log/perception')
Perception.on_lap(node,String(data=json.dumps({'run_id':'new','lap_id':1,'phase':'started'})))
assert not node.people and not node.plates and not node.events and not node.light_history and node.lap_active
Perception.on_status(node,String(data='COMPLETE old queued message'))
assert node.status=='START lap=1' and not node.completed
Perception.on_status(node,String(data='GO top_approach (1/17)'))
assert node.status.startswith('GO')
node.previous_report=saved
Perception.on_lap(node,String(data=json.dumps({'run_id':saved['run_id'],'lap_id':saved['lap_id'],'phase':'completed'})))
assert node.people==saved['people'] and node.plates==saved['plates'] and node.events==saved['events'] and node.completed and not node.lap_active
report={'passed':True,'new_lap_clears_counts_votes_and_light_history':True,'delayed_previous_complete_ignored':True,'completed_same_run_lap_recovers_own_saved_events':True,'method':'Call actual Perception lifecycle handlers with delayed recorded messages; no ROS topic publishing, model changes or scene answers'}
(root/'validation/patrol-cycle-review/lifecycle-replay.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
