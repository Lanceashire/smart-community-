#!/usr/bin/env python3
"""Generate offline capture rigs; never alter the official patrol world."""
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]

def main():
    out=ROOT/'log/validation-worlds';out.mkdir(parents=True,exist_ok=True)
    for scene in ['training','community']:
        for condition,ambient,diffuse in [('normal',.7,.9),('dim',.3,.45),('bright',.95,1.0)]:
            tree=ET.parse(ROOT/'src/community_demo/worlds'/f'{scene}.sdf');world=tree.getroot().find('world')
            world.find('scene/ambient').text=f'{ambient} {ambient} {ambient} 1'
            world.find('light/diffuse').text=f'{diffuse} {diffuse} {diffuse} 1'
            bot=next(m for m in world.findall('model') if m.get('name')=='patrol_bot')
            static=bot.find('static')
            if static is None:static=ET.SubElement(bot,'static')
            static.text='true'
            for link in bot.findall('link'):
                for collision in list(link.findall('collision')):link.remove(collision)
            for joint in bot.findall('joint'):joint.set('type','fixed')
            for plugin in list(bot.findall('plugin')):
                if 'OdometryPublisher' not in plugin.get('name',''):bot.remove(plugin)
            tree.write(out/f'{scene}-{condition}.sdf',encoding='unicode')
    print('Prepared six static, collision-free camera rigs')

if __name__=='__main__':main()
