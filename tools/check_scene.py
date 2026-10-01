"""Check source assets, actual SDF dimensions and collision setup."""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
package=ROOT/'src/community_demo'
assets=package/'models/community_assets'
manifest=json.loads((assets/'manifest.json').read_text())
layout=json.loads((package/'worlds/layout.json').read_text())
world=ET.parse(package/'worlds/community.sdf').getroot().find('world')
models={m.get('name'):m for m in world.findall('model')}
checks={}
checks['original_texture_sha256']=all(hashlib.sha256((assets/'materials/textures'/f'{a["id"]}.png').read_bytes()).hexdigest()==a['sha256'] for a in manifest['assets'])
checks['field_4p2_square']=models['ground'].findtext('.//collision/geometry/box/size')=='4.2 4.2 0.03'
checks['no_invented_boundary_walls']=not any('wall' in name for name in models)
checks['ten_reference_people']=len(layout['people'])==10 and all(p['name'] in models for p in layout['people'])
checks['three_metric_car_boards']=all(m.findtext('.//collision/geometry/box/size')=='.005 .345 .25' for n,m in models.items() if n.startswith('car_')) and len(layout['cars'])==3
checks['two_dimensioned_gantries']=all(models[n].findtext('.//collision/geometry/box/size')=='.015 .64 .14' for n in ['light_top','light_bottom'])
checks['twelve_lens_states']=len(layout['lamps'])==12 and all(l['name'] in models for l in layout['lamps'])
checks['camera_below_gate']=.265+.015<layout['gantry_m']['clearance']
checks['all_meshes_and_textures_resolve']=all((assets/'meshes'/uri.text.rsplit('/',1)[-1]).exists() for uri in world.findall('.//mesh/uri'))
checks['missing_assets_reported']=len(manifest['unreadable'])==2
report={'passed':all(checks.values()),'checks':checks,'asset_count':len(manifest['assets']),
        'missing_assets':manifest['unreadable'],'layout_assumptions':layout['assumptions'],
        'scope':'Static scene audit only. Does not prove navigation, recognition, or road compliance.'}
out=ROOT/'validation/scene-static-2026-10-01.json';out.parent.mkdir(exist_ok=True)
out.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
print(json.dumps(report,indent=2,ensure_ascii=False));raise SystemExit(0 if report['passed'] else 1)
