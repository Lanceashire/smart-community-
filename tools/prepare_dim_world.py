#!/usr/bin/env python3
"""Derive the dim patrol scene from current materials, changing lighting only."""
from pathlib import Path
import xml.etree.ElementTree as E

ROOT=Path(__file__).resolve().parents[1]

def prepare(source,output):
    tree=E.parse(source);world=tree.getroot().find('world')
    world.find('scene/ambient').text='.45 .45 .45 1'
    world.find('light/diffuse').text='.65 .65 .65 1'
    tree.write(output,encoding='utf-8',xml_declaration=True)

if __name__=='__main__':
    source=ROOT/'src/community_demo/worlds/community.sdf'
    prepare(source,source.with_name('community_dim.sdf'))
