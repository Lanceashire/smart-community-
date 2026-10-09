"""Import supplied AIC textures unchanged and generate metric Collada cards.

Usage: python3 tools/import_scene_assets.py /path/to/official-materials.zip
Requires Pillow at import time only. Runtime Gazebo does not need Pillow.
"""
import argparse
import hashlib
import io
import json
import math
from pathlib import Path
import zipfile
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / 'src/community_demo/models/community_assets'


def mesh(name, texture, width, height, *, silhouette=None, floor=False, disc=False, lit=False):
    """Local +X is the printed face; local +Z is up, floor normal is +Z."""
    vertices, uv, triangles = [], [], []
    def point(u, v):
        return ((u-.5)*width, (.5-v)*height, 0) if floor else (0, (u-.5)*width, (1-v)*height)
    def quad(u0, v0, u1, v1):
        k = len(vertices)
        for u, v in [(u0,v0),(u0,v1),(u1,v1),(u1,v0)]:
            vertices.append(point(u,v)); uv.append((u,1-v))
        triangles.extend([(k,k+1,k+2),(k,k+2,k+3)])
    if disc:
        vertices.append(point(.5,.5)); uv.append((.5,.5))
        for i in range(65):
            a=2*math.pi*i/64
            u,v=.5+.5*math.cos(a),.5+.5*math.sin(a)
            vertices.append(point(u,v)); uv.append((u,1-v))
        triangles=[(0,i+1,i) for i in range(1,65)]
    elif silhouette is not None:
        alpha=silhouette.getchannel('A')
        w,h=alpha.size
        for y in range(0,h,2):
            bounds=alpha.crop((0,y,w,min(y+2,h))).getbbox()
            if bounds:
                quad(bounds[0]/w,y/h,bounds[2]/w,min(y+2,h)/h)
    else:
        quad(0,0,1,1)
    flat=lambda xs:' '.join(str(round(v,8)) for x in xs for v in x)
    indices=' '.join(f'{i} {i} 0' for tri in triangles for i in tri)
    normal='0 0 1' if floor else '1 0 0'
    # The physical cardboard back is generated separately in the world.
    emission='<color>0 0 0 1</color>'  # Light emission is set in SDF, not an unsupported Collada texture.
    xml=f'''<?xml version="1.0" encoding="UTF-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
<asset><created>2026-10-01T00:00:00Z</created><modified>2026-10-01T00:00:00Z</modified><unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset>
<library_images><image id="image"><init_from>../materials/textures/{texture}</init_from></image></library_images>
<library_effects><effect id="effect"><profile_COMMON>
<newparam sid="surface"><surface type="2D"><init_from>image</init_from></surface></newparam>
<newparam sid="sampler"><sampler2D><source>surface</source><wrap_s>CLAMP</wrap_s><wrap_t>CLAMP</wrap_t></sampler2D></newparam>
<technique sid="common"><phong><emission>{emission}</emission><ambient><color>1 1 1 1</color></ambient>
<diffuse><texture texture="sampler" texcoord="UV"/></diffuse><specular><color>0 0 0 1</color></specular>
<transparent opaque="A_ONE"><texture texture="sampler" texcoord="UV"/></transparent><transparency><float>1</float></transparency>
</phong></technique></profile_COMMON></effect>
<effect id="back_effect"><profile_COMMON><technique sid="common"><lambert><diffuse><color>.8 .8 .8 1</color></diffuse></lambert></technique></profile_COMMON></effect></library_effects>
<library_materials><material id="front"><instance_effect url="#effect"/></material><material id="back"><instance_effect url="#back_effect"/></material></library_materials>
<library_geometries><geometry id="geometry"><mesh>
<source id="positions"><float_array id="positions_array" count="{len(vertices)*3}">{flat(vertices)}</float_array><technique_common><accessor source="#positions_array" count="{len(vertices)}" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
<source id="uv"><float_array id="uv_array" count="{len(uv)*2}">{flat(uv)}</float_array><technique_common><accessor source="#uv_array" count="{len(uv)}" stride="2"><param name="S" type="float"/><param name="T" type="float"/></accessor></technique_common></source>
<source id="normals"><float_array id="normals_array" count="3">{normal}</float_array><technique_common><accessor source="#normals_array" count="1" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
<vertices id="vertices"><input semantic="POSITION" source="#positions"/></vertices>
<triangles material="front" count="{len(triangles)}"><input semantic="VERTEX" source="#vertices" offset="0"/><input semantic="TEXCOORD" source="#uv" offset="1" set="0"/><input semantic="NORMAL" source="#normals" offset="2"/><p>{indices}</p></triangles>

</mesh></geometry></library_geometries>
<library_visual_scenes><visual_scene id="scene"><node id="card"><instance_geometry url="#geometry"><bind_material><technique_common>
<instance_material symbol="front" target="#front"><bind_vertex_input semantic="UV" input_semantic="TEXCOORD" input_set="0"/></instance_material>
<instance_material symbol="back" target="#back"/></technique_common></bind_material></instance_geometry></node></visual_scene></library_visual_scenes>
<scene><instance_visual_scene url="#scene"/></scene></COLLADA>'''
    (ASSETS/'meshes'/f'{name}.dae').write_text(xml)


def main():
    parser=argparse.ArgumentParser(); parser.add_argument('zip'); args=parser.parse_args()
    for path in ['meshes','materials/textures']:(ASSETS/path).mkdir(parents=True,exist_ok=True)
    manifest={'source_zip_sha256':hashlib.sha256(Path(args.zip).read_bytes()).hexdigest(),
              'assets':[], 'unreadable':[], 'assumptions':[
                  'Person height 0.15 m estimated from supplied ruler photograph; widths preserve source aspect.',
                  'Person cardboard thickness 0.002 m and lamp diameter 0.12 m are modelling choices.',
                  'Missing car background is replaced by explicitly generated thin front geometry.',
                  'Missing yellow-off lens uses a dark amber disc. No incomplete file was renamed to PNG.']}
    def member_path(z, member):
        # Both supplied archive versions contain the same assets; one has a root folder.
        matches=[name for name in (member, '复赛资料/'+member) if name in z.namelist()]
        if len(matches)>1:raise ValueError('Ambiguous duplicate asset: '+member)
        return matches[0] if matches else None
    def add(z,member,name,width,height,**kw):
        source=member_path(z,member)
        if source is None:raise FileNotFoundError('Missing official asset: '+member)
        data=z.read(source); im=Image.open(io.BytesIO(data)); im.load()
        filename=name+'.png'; (ASSETS/'materials/textures'/filename).write_bytes(data)
        if width is None:width=height*im.width/im.height
        mesh(name,filename,width,height,silhouette=im if kw.pop('silhouette',False) else None,**kw)
        manifest['assets'].append({'id':name,'source':member,'sha256':hashlib.sha256(data).hexdigest(),
                                  'pixels':list(im.size),'width_m':width,'height_m':height})
    with zipfile.ZipFile(args.zip) as z:
        add(z,'地图-图.png','floor',4.2,4.2,floor=True)
        for i in range(1,17):add(z,f'人员/社区人员/{i}.png',f'resident_{i:02}',None,.15,silhouette=True)
        for i in range(1,3):add(z,f'人员/非社区人员/F{i}.png',f'visitor_{i:02}',None,.15,silhouette=True)
        for i,n in enumerate(['一','二','三'],1):add(z,f'车辆识别/车牌{n}.png',f'plate_{i}',.095,.03)
        for name,ch in [('red','红'),('green','绿'),('yellow','黄')]:
            for state,suffix in [('on','亮'),('off','暗')]:
                member=f'红绿灯/红绿灯图片/{ch}灯{suffix}.png'
                if member_path(z,member) is not None:
                    add(z,member,f'{name}_{state}',.12,.12,disc=True,lit=state=='on')
        for member in z.namelist():
            if member.startswith('__MACOSX/') or Path(member).name.startswith('._'):
                continue
            if member.endswith('.downloading'):
                try:Image.open(io.BytesIO(z.read(member))).load()
                except Exception as ex:manifest['unreadable'].append({'source':member,'error':type(ex).__name__})
    (ASSETS/'manifest.json').write_text(json.dumps(manifest,indent=2,ensure_ascii=False)+'\n')
    (ASSETS/'model.config').write_text('<model><name>community_assets</name><version>1.0</version><sdf version="1.9">model.sdf</sdf><description>Official AIC cards, unchanged source textures</description></model>')
    (ASSETS/'model.sdf').write_text('<sdf version="1.9"><model name="community_assets"><static>true</static><link name="assets"/></model></sdf>')
    print(f'Imported {len(manifest["assets"])} assets; {len(manifest["unreadable"])} incomplete files; {ASSETS}')


if __name__=='__main__':main()
