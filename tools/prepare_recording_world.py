#!/usr/bin/env python3
"""Add a read-only overhead Gazebo camera. No collision, map or controller changes."""
from pathlib import Path
import xml.etree.ElementTree as E
root=Path(__file__).resolve().parents[1]
p=root/'src/community_demo/worlds/community.sdf';tree=E.parse(p);world=tree.getroot().find('world')
model=E.fromstring('''<model name="recording_camera"><static>true</static><pose>0 0 5.3 0 1.5707963267948966 -1.5707963267948966</pose><link name="camera"><sensor name="overview" type="camera"><always_on>true</always_on><update_rate>10</update_rate><topic>/recording/overview</topic><camera><horizontal_fov>0.9</horizontal_fov><image><width>1200</width><height>1200</height><format>R8G8B8</format></image><clip><near>0.1</near><far>15</far></clip></camera></sensor></link></model>''')
world.append(model)
out=p.with_name('community_recording.sdf');tree.write(out,encoding='utf-8',xml_declaration=True)
print(out)
