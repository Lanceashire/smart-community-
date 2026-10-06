#!/usr/bin/env python3
"""Known-field planning grid including physical signal posts, without changing the world."""
import json,math,sys,xml.etree.ElementTree as ET
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/community_demo'))
from community_demo.road_rules import in_road

def build(destination=None):
    destination=Path(destination) if destination else ROOT/'src/community_demo/maps'
    posts=[]
    for model in ET.parse(ROOT/'src/community_demo/worlds/community.sdf').findall('.//world/model'):
        if model.attrib['name'] not in ('light_top','light_bottom'):continue
        mx,my,mz,mr,mp,ma=map(float,model.findtext('pose').split());c,s=math.cos(ma),math.sin(ma)
        for collision in model.findall('./link/collision'):
            if not collision.attrib['name'].startswith('leg_'):continue
            x,y,z,rr,pp,a=map(float,collision.findtext('pose').split());sx,sy,sz=map(float,collision.findtext('geometry/box/size').split())
            posts.append({'name':model.attrib['name']+'/'+collision.attrib['name'],'x':mx+c*x-s*y,'y':my+s*x+c*y,'yaw':ma+a,'width':sx,'depth':sy})
    size=210;resolution=.02;pixels=bytearray(size*size);occupied=0
    for row in range(size):
        for col in range(size):
            x=-2.1+(col+.5)*resolution;y=2.1-(row+.5)*resolution
            free=in_road(x,y,.012)
            for p in posts:
                c,s=math.cos(p['yaw']),math.sin(p['yaw']);dx=x-p['x'];dy=y-p['y']
                if abs(c*dx+s*dy)<=p['width']/2+resolution/math.sqrt(2)+.02 and abs(-s*dx+c*dy)<=p['depth']/2+resolution/math.sqrt(2)+.02:
                    if free:occupied+=1
                    free=False;break
            pixels[row*size+col]=254 if free else 0
    destination.mkdir(parents=True,exist_ok=True)
    (destination/'roads.pgm').write_bytes(b'P5\n210 210\n255\n'+pixels)
    (destination/'roads.yaml').write_text('image: roads.pgm\nmode: trinary\nresolution: 0.02\norigin: [-2.1, -2.1, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    evidence={'source':'Known supplied field geometry: drivable corridors plus physical gantry collision legs','posts':posts,'added_occupied_road_cells':occupied,'resolution':resolution,'planning_post_margin_m':.02,'physical_world_modified':False,'overhead_panels':'Not occupied at robot clearance height'}
    (destination/'planning-obstacles.json').write_text(json.dumps(evidence,indent=2)+'\n')
    print(json.dumps(evidence));return evidence
if __name__=='__main__':build()
