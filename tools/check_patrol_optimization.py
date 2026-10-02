#!/usr/bin/env python3
"""Regression checks for late-green release, lost sensor data and bounded retry."""
import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
from unittest.mock import patch

import numpy as np
from rclpy.time import Time
from community_demo.signal_gate import FreshGreenGate
from community_demo.patrol import Patrol,OBSERVATION_HEADING

gate=FreshGreenGate()
for _ in range(20):gate.observe('green')
assert not gate.ready
for c in ['yellow','yellow','unknown','red','red','green','green','green','green']:gate.observe(c)
assert not gate.ready
gate.observe('green');assert gate.ready
gate.invalidate()
for _ in range(10):gate.observe('green')
assert not gate.ready
gate.observe('red');gate.observe('red')
for _ in range(5):gate.observe('green')
assert gate.ready

published=[];warnings=[]
node=SimpleNamespace(observation_align=True,observation_start=Time(seconds=1),observation_index=0,
    observation_attempt=0,pose=(1.1,-1.7,OBSERVATION_HEADING['observe_plate_1']),
    pan_pub=SimpleNamespace(publish=lambda m:published.append(m.data)),
    observation_pub=SimpleNamespace(publish=lambda m:None),inspection_pub=SimpleNamespace(publish=lambda m:None),
    run_id='offline-regression',lap_id=1,saved_views=[],observation_outcomes=[],
    observation_feedback={'view':'plate_1','ready':False},feedback_update=Time(seconds=7),
    camera_frame=np.zeros((100,100,3),np.uint8),publish_command=lambda *a:None,
    log_status=lambda *a:None,get_logger=lambda:SimpleNamespace(warning=lambda s:warnings.append(s)))
with tempfile.TemporaryDirectory() as folder,patch('community_demo.patrol.Path',lambda p:Path(folder)/p):
    assert not Patrol.observe(node,'observe_plate_1',Time(seconds=7))
    assert node.observation_attempt==1 and abs(published[-1]+1.49079632679)<.001
    node.feedback_update=Time(seconds=12)
    assert not Patrol.observe(node,'observe_plate_1',Time(seconds=12))
    assert node.observation_attempt==2
    node.feedback_update=Time(seconds=15)
    assert Patrol.observe(node,'observe_plate_1',Time(seconds=15))
    assert node.observation_outcomes[-1]['confirmed'] is False and warnings
    node.observation_start=Time(seconds=20);node.observation_feedback={'view':'plate_1','ready':True};node.feedback_update=Time(seconds=26)
    assert Patrol.observe(node,'observe_plate_1',Time(seconds=26))
    assert node.observation_outcomes[-1]['confirmed'] is True

report={'passed':True,'late_green_does_not_release':True,'fresh_transition_requires_five_green_samples':True,
    'sensor_outage_requires_new_transition':True,'two_pan_retries_before_timeout':True,
    'timeout_is_explicitly_unconfirmed':True,'successful_view_completes':True,
    'scope':'Offline actual controller methods; complements live physical laps; no injected ROS answers'}
root=Path(__file__).resolve().parents[1]
(root/'validation/patrol-optimization-2026-10-02/controller-regression.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report))
