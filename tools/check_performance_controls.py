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
    node.observation_start=Time(seconds=30);node.feedback_update=Time(seconds=31)
    assert not Patrol.observe(node,'observe_plate_1',Time(seconds=31))
    node.feedback_update=Time(seconds=33)
    assert Patrol.observe(node,'observe_plate_1',Time(seconds=33))
    assert node.observation_outcomes[-1]['elapsed_s']==3
    node.observation_start=Time(seconds=40);node.feedback_update=Time(seconds=40)
    assert not Patrol.observe(node,'observe_plate_1',Time(seconds=44))  # stale confirmation cannot release

report={'passed':True,'late_green_does_not_release':True,'fresh_transition_requires_five_green_samples':True,
    'sensor_outage_requires_new_transition':True,'two_pan_retries_before_timeout':True,
    'timeout_is_explicitly_unconfirmed':True,'successful_view_completes':True,
    'scope':'Offline actual controller methods; complements live physical laps; no injected ROS answers'}
root=Path(__file__).resolve().parents[1]


# Invalid camera pixels must not count as a successful registration, even while
# the camera and encoder streams themselves are fresh.
from collections import deque
from community_demo.localization import Localization
from sensor_msgs.msg import Image,LaserScan
clock=SimpleNamespace(now=lambda:Time(seconds=10))
blank=Image();blank.header.stamp=Time(seconds=10).to_msg();blank.encoding='bgr8'
blank.height=96;blank.width=128;blank.step=384;blank.data=np.zeros((96,128,3),np.uint8).tobytes()
local=SimpleNamespace(get_clock=lambda:clock,camera_received=0.,frame_time=0.,pan_samples=deque([(10.,0.,0.)]),last_match=5.)
Localization.image(local,blank)
assert local.last_match==5. and local.registration_points==0 and local.encoder_age==0.
quality=[];poses=[]
local=SimpleNamespace(get_clock=lambda:clock,odom_time=blank.header.stamp,odom_received=10.,camera_received=10.,
    unmatched_distance=0.,last_match=9.,encoder_age=0.,score=.01,registration_points=100,
    xy=np.array([1.1,1.7]),yaw=3.14159,quality=SimpleNamespace(publish=lambda m:quality.append(json.loads(m.data))),
    tf=SimpleNamespace(sendTransform=lambda m:None),poses=SimpleNamespace(publish=lambda m:poses.append(m)),status=SimpleNamespace(publish=lambda m:None))
for distance,state in [(.1,'tracking'),(.4,'degraded'),(.8,'lost')]:
    local.unmatched_distance=distance;Localization.publish(local);assert quality[-1]['state']==state
scan_node=SimpleNamespace(get_clock=lambda:clock,clear_scans=0,scan=None)
scan_node.front_blocked=lambda:Patrol.front_blocked(scan_node)
scan=LaserScan();scan.angle_min=-.2;scan.angle_increment=.2;scan.ranges=[6.,.2,6.]
Patrol.on_scan(scan_node,scan);assert scan_node.clear_scans==0 and scan_node.front_blocked()
scan.ranges=[6.,1.,6.]
for _ in range(4):Patrol.on_scan(scan_node,scan)
assert scan_node.clear_scans==4
Patrol.on_scan(scan_node,scan);assert scan_node.clear_scans==5
for encoder in [(10.,1.57,0.),(10.,0.,.3),(9.7,0.,0.)]:
    gated=SimpleNamespace(get_clock=lambda:clock,camera_received=0.,frame_time=0.,pan_samples=deque([encoder]),last_match=5.,registration_points=123)
    Localization.image(gated,blank)
    assert gated.frame_time==0. and gated.last_match==5. and gated.registration_points==123

from community_demo.road_rules import clear_of_posts
posts=json.loads((root/'src/community_demo/maps/planning-obstacles.json').read_text())['posts']
assert len(posts)==4
assert not clear_of_posts(.2075,-1.70,0.,posts)
assert clear_of_posts(.2075,-1.82,0.,posts)
assert clear_of_posts(.2075,-1.82,.785398,posts)

report.update(static_post_collision_guard=True,registration_view_guard=True,blank_image_does_not_refresh_registration=True,localization_quality_states=True,clear_scan_debounce=True,early_finish_and_stale_feedback_guard=True)
(root/'validation/performance-2026-10-02/controller-regression.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report))
