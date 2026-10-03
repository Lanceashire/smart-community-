#!/usr/bin/env python3
"""Exercise actual approach callbacks and timed release history without ROS injection."""
import json
from pathlib import Path
from types import SimpleNamespace, MethodType
from rclpy.time import Time
from std_msgs.msg import String
from community_demo.patrol import Patrol
from community_demo.signal_gate import FreshGreenGate

clock = [0.0]
node = SimpleNamespace(index=0, completed=False, signal_context=None,
                       signal_gate=FreshGreenGate(), waiting_for_green=False,
                       get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=clock[0])))
node.update_signal_context = MethodType(Patrol.update_signal_context, node)

def observe(t, colour):
    clock[0] = t
    Patrol.on_light(node, String(data=colour))

observe(0., 'red'); observe(.2, 'red')
for i in range(5): observe(.4 + i * .2, 'green')
assert node.signal_gate.ready and not node.waiting_for_green
node.index = 1
node.update_signal_context()
assert node.signal_gate.ready  # Entering STOP keeps the approach transition.
node.index = 2; node.update_signal_context()
assert node.signal_context is None and not node.signal_gate.ready
node.index = 9
for i in range(10): observe(2. + i * .2, 'green')
assert not node.signal_gate.ready  # The next junction cannot reuse old evidence.
observe(4., 'red'); observe(4.2, 'red')
for i in range(5): observe(4.4 + i * .2, 'green')
assert node.signal_gate.ready
observe(6.2, 'green')  # A one-second stream gap invalidates the transition.
assert not node.signal_gate.ready
observe(6.4, 'red'); observe(6.6, 'red')
for i in range(5): observe(6.8 + i * .2, 'green')
assert node.signal_gate.ready
for i in range(6, 30): observe(6.8 + i * .2, 'green')
assert not node.signal_gate.ready  # Remembered green is now too old.
observe(12.8, 'unknown')
for i in range(5): observe(13. + i * .2, 'green')
assert not node.signal_gate.ready  # Unknown cannot renew an old green window.
observe(14., 'yellow'); observe(14.2, 'yellow')
for i in range(5): observe(14.4 + i * .2, 'green')
assert node.signal_gate.ready
node.completed = True; node.update_signal_context()
assert not node.signal_gate.ready
result = {'passed': True, 'approach_to_stop_retains_transition': True,
          'new_junction_resets_history': True, 'unknown_age_green_does_not_release': True,
          'sensor_gap_requires_new_transition': True, 'old_green_window_expires': True,
          'unknown_does_not_renew_green': True, 'completion_resets_history': True,
          'scope': 'Actual controller callbacks, no injected ROS traffic states or answers'}
path = Path(__file__).resolve().parents[1] / 'validation/signal-optimization-2026-10-03/controller-checks.json'
path.write_text(json.dumps(result, indent=2) + '\n')
print(json.dumps(result))
