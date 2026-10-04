"""Build the AIC semifinal scene on the existing RM-derived patrol baseline.

Textures are copied unchanged from the user's ZIP. Inferred positions and the
replacement for two incomplete assets are recorded in layout/asset manifests.
"""

from pathlib import Path
import json
import math

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'src/community_demo/worlds/community.sdf'


def colour(r, g, b, a=1):
    return f'{r} {g} {b} {a}'


def box(name, x, y, z, sx, sy, sz, rgba, *, collision=False):
    geometry = f'<box><size>{sx} {sy} {sz}</size></box>'
    inert = '<inertial><mass>1</mass><inertia><ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz></inertia></inertial>'
    coll = f'<collision name="collision"><geometry>{geometry}</geometry></collision>' if collision else ''
    return f'''<model name="{name}"><static>true</static><pose>{x} {y} {z} 0 0 0</pose>
      <link name="body">{inert}{coll}<visual name="visual"><geometry>{geometry}</geometry>
      <material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse></material></visual></link></model>'''


def mesh_visual(name, asset, pose='0 0 0 0 0 0'):
    lit_rgb={'red':'.55 .015 .015 1','green':'.015 .55 .015 1','yellow':'.55 .35 .015 1'}
    emission=f'<emissive>{lit_rgb[asset.split("_")[0]]}</emissive>' if asset.endswith('_on') else ''
    return f'<visual name="{name}"><pose>{pose}</pose><geometry><mesh><uri>model://community_assets/meshes/{asset}.dae</uri></mesh></geometry><material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>{emission}<pbr><metal><albedo_map>model://community_assets/materials/textures/{asset}.png</albedo_map><metalness>0</metalness><roughness>1</roughness></metal></pbr></material></visual>'


def part(name, pose, size, rgba, collision=False):
    geometry=f'<geometry><box><size>{size}</size></box></geometry>'
    coll=f'<collision name="{name}_collision"><pose>{pose}</pose>{geometry}</collision>' if collision else ''
    return f'{coll}<visual name="{name}"><pose>{pose}</pose>{geometry}<material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse></material></visual>'


def card(name, asset, x, y, yaw, width):
    back=part('cardboard', '-.001 0 .075 0 0 0', f'.002 {width} .15', '.85 .85 .85 1', True)
    base=part('foot', '0 0 .002 0 0 0', f'.025 {width} .004', '.22 .22 .22 1', True)
    return f'<model name="{name}"><static>true</static><pose>{x} {y} .004 0 0 {yaw}</pose><link name="body">{back}{base}{mesh_visual("portrait",asset,".0002 0 0 0 0 0")}</link></model>'


def gantry(name, x, y, yaw, reverse=False):
    parts=part('panel','0 0 .41 0 0 0','.015 .64 .14','.025 .025 .03 1',True)
    for side in [-1,1]:
        parts+=part(f'leg_{side}',f'0 {side*.3075} .17 0 0 0','.025 .025 .34','.18 .18 .19 1',True)
    models=[f'<model name="{name}"><static>true</static><pose>{x} {y} 0 0 0 {yaw}</pose><link name="body">{parts}</link></model>']
    lenses=[]
    for colour_, lateral in [('red',.20),('yellow',0),('green',-.20)]:
        if reverse:lateral=-lateral
        lx=x+.0085*math.cos(yaw)-lateral*math.sin(yaw)
        ly=y+.0085*math.sin(yaw)+lateral*math.cos(yaw)
        for state in ['on','off']:
            identifier=f'{name}_{colour_}_{state}'
            active=(state=='on')==(colour_=='red')
            z=.35 if active else -1.0
            visual=mesh_visual('lens',f'{colour_}_{state}') if not (colour_=='yellow' and state=='off') else '<visual name="lens"><pose>0 0 .06 0 1.5707963 0</pose><geometry><cylinder><radius>.06</radius><length>.001</length></cylinder></geometry><material><ambient>.08 .055 .008 1</ambient><diffuse>.08 .055 .008 1</diffuse></material></visual>'
            models.append(f'<model name="{identifier}"><static>true</static><pose>{lx} {ly} {z} 0 0 {yaw}</pose><link name="lens">{visual}</link></model>')
            lenses.append({'name':identifier,'colour':colour_,'state':state,'x':lx,'y':ly,'yaw':yaw,'visible_z':.35,'hidden_z':-1.0})
    return models,lenses


def car(i,y):
    # The ZIP's car background is incomplete: this visibly separate thin panel
    # is a geometric replacement, not an invented official texture.
    parts=part('board','0 0 .125 0 0 0','.005 .345 .25','.04 .62 .7 1',True)
    parts+=part('windscreen','.003 0 .197 0 0 0','.001 .27 .067','.04 .07 .10 1')
    parts+=part('grille','.003 0 .112 0 0 0','.001 .17 .035','.025 .025 .025 1')
    parts+=part('bumper','.003 0 .018 0 0 0','.001 .30 .025','.15 .16 .18 1')
    for j in [-1,1]:
        parts+=part(f'headlight_{j}',f'.003 {j*.132} .115 0 0 0','.001 .048 .038','.95 .94 .75 1')
    parts+=mesh_visual('official_plate',f'plate_{i}','.0038 0 .040 0 0 0')
    return f'<model name="car_{i}"><static>true</static><pose>1.75 {y} .004 0 0 {math.pi}</pose><link name="body">{parts}</link></model>'


def robot():
    return '''<model name="patrol_bot"><pose>1.1 1.7 .002 0 0 3.14159265</pose>
      <link name="chassis"><pose>0 0 .13 0 0 0</pose>
        <inertial><mass>1.5</mass><inertia><ixx>.012</ixx><iyy>.012</iyy><izz>.020</izz></inertia></inertial>
        <collision name="body_collision"><geometry><box><size>.22 .16 .12</size></box></geometry></collision>
        <visual name="body_visual"><geometry><box><size>.22 .16 .12</size></box></geometry>
          <material><ambient>.08 .25 .95 1</ambient><diffuse>.08 .25 .95 1</diffuse></material></visual>
        <!-- Front/rear low-friction spherical supports prevent pitching on the drive axle. -->
        <collision name="front_support"><pose>.085 0 -.105 0 0 0</pose>
          <geometry><sphere><radius>.025</radius></sphere></geometry>
          <surface><friction><ode><mu>0</mu><mu2>0</mu2></ode></friction></surface></collision>
        <collision name="rear_support"><pose>-.085 0 -.105 0 0 0</pose>
          <geometry><sphere><radius>.025</radius></sphere></geometry>
          <surface><friction><ode><mu>0</mu><mu2>0</mu2></ode></friction></surface></collision>
        <visual name="front_support_visual"><pose>.085 0 -.105 0 0 0</pose>
          <geometry><sphere><radius>.025</radius></sphere></geometry></visual>
        <visual name="rear_support_visual"><pose>-.085 0 -.105 0 0 0</pose>
          <geometry><sphere><radius>.025</radius></sphere></geometry></visual>
        <visual name="roof"><pose>0 0 .067 0 0 0</pose>
          <geometry><box><size>.18 .14 .015</size></box></geometry>
          <material><ambient>.95 .08 .55 1</ambient><diffuse>.95 .08 .55 1</diffuse></material></visual>
        <visual name="front_marker"><pose>.085 0 .077 0 0 0</pose>
          <geometry><box><size>.035 .13 .018</size></box></geometry>
          <material><ambient>1 .85 0 1</ambient><diffuse>1 .85 0 1</diffuse></material></visual>
        <visual name="lidar_housing"><pose>-.03 0 .085 0 0 0</pose>
          <geometry><cylinder><radius>.025</radius><length>.025</length></cylinder></geometry>
          <material><ambient>.08 .08 .1 1</ambient><diffuse>.08 .08 .1 1</diffuse></material></visual>
        <!-- 0.235 m: below 0.254 m car-board top; no artificial perimeter walls. -->
        <sensor name="laser" type="gpu_lidar"><topic>/lidar/raw</topic><pose>0 0 .105 0 0 0</pose>
          <always_on>true</always_on><update_rate>10</update_rate><ray><scan><horizontal>
          <samples>720</samples><resolution>1</resolution><min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle></horizontal><vertical><samples>16</samples><resolution>1</resolution><min_angle>-.60</min_angle><max_angle>.10</max_angle></vertical></scan><range><min>.08</min><max>6</max></range>
          </ray></sensor>
      </link>
      <link name="camera_head"><pose>0 0 .265 0 0 0</pose>
        <inertial><mass>.03</mass><inertia><ixx>.00004</ixx><iyy>.00004</iyy><izz>.00004</izz></inertia></inertial>
        <visual name="housing"><geometry><box><size>.05 .05 .03</size></box></geometry>
          <material><ambient>.035 .04 .05 1</ambient><diffuse>.035 .04 .05 1</diffuse></material></visual>
        <visual name="lens"><pose>.026 0 0 0 1.5707963 0</pose><geometry><cylinder><radius>.009</radius><length>.003</length></cylinder></geometry>
          <material><ambient>.02 .15 .23 1</ambient><diffuse>.02 .15 .23 1</diffuse></material></visual>
        <sensor name="camera" type="camera"><topic>/camera/image</topic><pose>.03 0 0 0 .10 0</pose>
          <always_on>true</always_on><update_rate>10</update_rate>
          <camera><horizontal_fov>1.6</horizontal_fov><image><width>1280</width><height>960</height>
            <format>R8G8B8</format></image><clip><near>.03</near><far>6</far></clip></camera></sensor>
      </link>
      <joint name="camera_pan_joint" type="revolute"><parent>chassis</parent><child>camera_head</child>
        <axis><xyz expressed_in="__model__">0 0 1</xyz><limit><lower>-3.14159265</lower><upper>3.14159265</upper></limit><dynamics><damping>.01</damping></dynamics></axis></joint>
      <plugin filename="gz-sim-joint-position-controller-system" name="gz::sim::systems::JointPositionController">
        <joint_name>camera_pan_joint</joint_name><topic>/model/patrol_bot/camera_pan</topic>
        <p_gain>.2</p_gain><d_gain>.02</d_gain><cmd_max>.15</cmd_max><cmd_min>-.15</cmd_min>
      </plugin>
      <link name="left_wheel"><pose>0 .10 .055 1.5707963 0 0</pose>
        <inertial><mass>.15</mass><inertia><ixx>.0003</ixx><iyy>.0003</iyy><izz>.0003</izz></inertia></inertial>
        <collision name="collision"><geometry><cylinder><radius>.055</radius><length>.025</length></cylinder></geometry>
          <surface><friction><ode><mu>1</mu><mu2>1</mu2></ode></friction></surface></collision>
        <visual name="visual"><geometry><cylinder><radius>.055</radius><length>.025</length></cylinder></geometry>
          <material><ambient>.02 .02 .02 1</ambient><diffuse>.02 .02 .02 1</diffuse></material></visual>
      </link>
      <link name="right_wheel"><pose>0 -.10 .055 1.5707963 0 0</pose>
        <inertial><mass>.15</mass><inertia><ixx>.0003</ixx><iyy>.0003</iyy><izz>.0003</izz></inertia></inertial>
        <collision name="collision"><geometry><cylinder><radius>.055</radius><length>.025</length></cylinder></geometry>
          <surface><friction><ode><mu>1</mu><mu2>1</mu2></ode></friction></surface></collision>
        <visual name="visual"><geometry><cylinder><radius>.055</radius><length>.025</length></cylinder></geometry>
          <material><ambient>.02 .02 .02 1</ambient><diffuse>.02 .02 .02 1</diffuse></material></visual>
      </link>
      <joint name="left_wheel_joint" type="revolute"><parent>chassis</parent><child>left_wheel</child>
        <axis><xyz expressed_in="__model__">0 1 0</xyz>
          <limit><lower>-1e16</lower><upper>1e16</upper></limit></axis></joint>
      <joint name="right_wheel_joint" type="revolute"><parent>chassis</parent><child>right_wheel</child>
        <axis><xyz expressed_in="__model__">0 1 0</xyz>
          <limit><lower>-1e16</lower><upper>1e16</upper></limit></axis></joint>
      <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
        <left_joint>left_wheel_joint</left_joint><right_joint>right_wheel_joint</right_joint>
        <wheel_separation>.20</wheel_separation><wheel_radius>.055</wheel_radius>
        <topic>/model/patrol_bot/cmd_vel</topic><odom_topic>/model/patrol_bot/odometry</odom_topic>
        <frame_id>odom</frame_id><child_frame_id>base_link</child_frame_id>
        <odom_publish_frequency>20</odom_publish_frequency>
      </plugin>
      <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
        <dimensions>3</dimensions><odom_frame>world</odom_frame><robot_base_frame>base_link</robot_base_frame>
        <odom_topic>/model/patrol_bot/ground_truth</odom_topic><odom_publish_frequency>20</odom_publish_frequency>
        <tf_topic>/model/patrol_bot/ground_truth_tf</tf_topic>
      </plugin>
      <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher"/>
    </model>'''


def main():
    assets=ROOT/'src/community_demo/models/community_assets/manifest.json'
    manifest=json.loads(assets.read_text())
    widths={a['id']:a['width_m'] for a in manifest['assets']}
    models=[box('ground',0,0,-.015,4.2,4.2,.03,colour(.03,.03,.03),collision=True),
            '<model name="official_map"><static>true</static><pose>0 0 .001 0 0 0</pose><link name="floor">'+mesh_visual('map','floor')+'</link></model>']
    people=[]
    # Ten cards reproduce the schematic's five-A/five-B arrangement. Identity
    # assignments are configurable examples; the task specifies no exact IDs.
    reference=json.loads((OUT.parent/'people_reference.json').read_text())
    angles={'east':0,'north':math.pi/2,'west':math.pi,'south':-math.pi/2}
    for item in reference['people']:
        asset=item['asset'];x=item['x'];y=item['y'];yaw=angles[item['facing']]
        models.append(card(item['name'],asset,x,y,yaw,widths[asset]))
        people.append({**item,'yaw':yaw,'height_m':.15,'width_m':widths[asset]})
    lamps=[]
    for name,x,y,yaw,reverse in [('light_top',-.3,1.7,0,False),('light_bottom',-.1,-1.62,math.pi/2,True)]:
        gantry_models,lenses=gantry(name,x,y,yaw,reverse);models.extend(gantry_models);lamps.extend(lenses)
    for i,y in enumerate([-1.7,-1.1,-.5],1):models.append(car(i,y))
    models.append(robot())
    body='\n'.join(models)
    sdf=f'''<?xml version="1.0"?>
<sdf version="1.9"><world name="community">
  <gravity>0 0 -9.8</gravity><physics name="physics" type="ignored"><max_step_size>.001</max_step_size><real_time_factor>1</real_time_factor></physics>
  <scene><ambient>.7 .7 .7 1</ambient><background>.12 .15 .20 1</background><shadows>true</shadows><grid>false</grid></scene>
  <light name="sun" type="directional"><pose>0 0 5 0 0 0</pose><diffuse>.9 .9 .9 1</diffuse><specular>.1 .1 .1 1</specular><direction>-.3 -.4 -1</direction></light>
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
  <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
  <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
  <plugin filename="libcommunity_signal_system.so" name="community::SignalSystem"><red_seconds>10</red_seconds><green_seconds>15</green_seconds><yellow_seconds>3</yellow_seconds></plugin>
  {body}
</world></sdf>'''
    OUT.parent.mkdir(parents=True,exist_ok=True);OUT.write_text(sdf)
    layout={'field_m':[4.2,4.2],'road_width_m':.6,'map_texture':'original supplied map; no invented perimeter wall',
            'origin':'map centre; +X right, +Y top, +Z up','spawn':[1.1,1.7,math.pi],
            'people':people,'lamps':lamps,'cars':[{'name':f'car_{i}','x':1.75,'y':y,'yaw':math.pi,'size_m':[.005,.345,.25],'plate_m':[.095,.03]} for i,y in enumerate([-1.7,-1.1,-.5],1)],
            'gantry_m':{'width':.64,'height':.48,'clearance':.34,'panel_height':.14,'leg_width':.025},
            'stop_lines':{'top_x':.30,'bottom_y':-.51},
            'assumptions':['Coordinates inferred from the provided raster map and snapped to nominal 0.1 m; not a surveyed CAD drawing.',
                           'Five A and five B people follow schematic; identities and their exact positions are configurable.',
                           'Lower gantry y=-1.62 m leaves room for the differential robot turning south of the posts.',
                           'Start is on the driving lane beside the upper-right start/finish box, not inside the parking column.']}
    OUT.with_name('layout.json').write_text(json.dumps(layout,indent=2,ensure_ascii=False)+'\n')
    print(OUT)


if __name__=='__main__':main()
