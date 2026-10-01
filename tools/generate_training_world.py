#!/usr/bin/env python3
"""Separate training scene covers all 18 supplied identity standees."""
import json,math,xml.etree.ElementTree as ET
from pathlib import Path
from generate_world import card
root=Path(__file__).resolve().parents[1]/'src/community_demo';layout=json.loads((root/'worlds/layout.json').read_text());tree=ET.parse(root/'worlds/community.sdf');world=tree.getroot().find('world')
for m in list(world.findall('model')):
    if m.attrib['name'].startswith(('A_person','B_person')):world.remove(m)
manifest=json.loads((root/'models/community_assets/manifest.json').read_text());widths={a['id']:a['width_m'] for a in manifest['assets']};people=[]
assets=[f'resident_{i:02}' for i in range(1,17)]+['visitor_01','visitor_02']
for i,asset in enumerate(assets):
    if i<9:x=-1.25+(i%9)*.14;y=1.05;yaw=math.pi if i==8 else (math.pi/2 if i%2 else -math.pi/2)
    else:j=i-9;x=-1.19+(j%5)*.155;y=-.11-(j//5)*.22;yaw=0 if j==4 else math.pi/2
    name=f'training_person_{i}';world.append(ET.fromstring(card(name,asset,x,y,yaw,widths[asset])));people.append({'name':name,'asset':asset,'x':x,'y':y,'yaw':yaw,'width_m':widths[asset],'height_m':.15})
layout['people']=people;tree.write(root/'worlds/training.sdf',encoding='unicode');(root/'worlds/training-layout.json').write_text(json.dumps(layout,indent=2));print('Separate training scene:',len(people),'identities')
