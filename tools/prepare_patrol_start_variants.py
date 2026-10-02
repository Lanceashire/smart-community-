#!/usr/bin/env python3
"""Change only initial spawn pose; runtime localization retains its nominal prior."""
import hashlib,json,math,xml.etree.ElementTree as ET
from pathlib import Path

root=Path(__file__).resolve().parents[1];worlds=root/'src/community_demo/worlds'
base=worlds/'community.sdf';manifest=[]
for name,dx,dy,da in [('start-plus',.025,-.025,.035),('start-minus',-.025,.025,-.035)]:
    tree=ET.parse(base);robot=tree.getroot().find("world/model[@name='patrol_bot']")
    robot.find('pose').text=f'{1.1+dx} {1.7+dy} .002 0 0 {math.pi+da}'
    destination=worlds/f'audit-{name}.sdf';tree.write(destination,encoding='utf-8',xml_declaration=True)
    manifest.append({'name':name,'world':destination.name,'offset_m':[dx,dy],'yaw_offset_rad':da,
                     'start_pose':[1.1+dx,1.7+dy,math.pi+da],
                     'sha256':hashlib.sha256(destination.read_bytes()).hexdigest()})
record={'base_world_sha256':hashlib.sha256(base.read_bytes()).hexdigest(),'variants':manifest,
        'scope':'Initial pose variation only; no teleport after start, no answers supplied to localization or vision'}
(root/'validation/patrol-optimization-2026-10-02/start-variants.json').write_text(json.dumps(record,indent=2)+'\n')
print(json.dumps(record))
