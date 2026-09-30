"""Generate the compact Gazebo Harmonic proof-of-run world.

The 4.2 m square and route topology follow the supplied AIC semifinal sketch.
Small coloured standees and blank plates are explicit first-run placeholders.
"""

from pathlib import Path

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


def standee(name, x, y, outsider=False):
    rgb = colour(.95, .30, .10) if outsider else colour(.10, .45, .88)
    return box(name, x, y, .15, .08, .13, .30, rgb, collision=True)


def traffic_light(name, x, y, yaw):
    # Green is fixed in this first-run world; the image classifier still uses
    # camera frames rather than simulator state. Dynamic phases are future work.
    return f'''<model name="{name}"><static>true</static><pose>{x} {y} 0.47 0 0 {yaw}</pose>
      <link name="body">
        <visual name="panel"><geometry><box><size>.045 .64 .18</size></box></geometry>
          <material><ambient>.02 .02 .02 1</ambient><diffuse>.02 .02 .02 1</diffuse></material></visual>
        <visual name="red"><pose>.026 .20 0 0 1.5708 0</pose>
          <geometry><cylinder><radius>.06</radius><length>.008</length></cylinder></geometry>
          <material><ambient>.10 .01 .01 1</ambient><diffuse>.10 .01 .01 1</diffuse></material></visual>
        <visual name="yellow"><pose>.026 0 0 0 1.5708 0</pose>
          <geometry><cylinder><radius>.06</radius><length>.008</length></cylinder></geometry>
          <material><ambient>.12 .08 .01 1</ambient><diffuse>.12 .08 .01 1</diffuse></material></visual>
        <visual name="green"><pose>.026 -.20 0 0 1.5708 0</pose>
          <geometry><cylinder><radius>.06</radius><length>.008</length></cylinder></geometry>
          <material><ambient>0 .9 .02 1</ambient><diffuse>0 .9 .02 1</diffuse>
            <emissive>0 1 0 1</emissive></material></visual>
      </link></model>'''


def robot():
    return '''<model name="patrol_bot"><pose>1.65 1.55 0 0 0 3.14159265</pose>
      <link name="chassis"><pose>0 0 .13 0 0 0</pose>
        <inertial><mass>1.5</mass><inertia><ixx>.012</ixx><iyy>.012</iyy><izz>.020</izz></inertia></inertial>
        <collision name="body_collision"><geometry><box><size>.22 .16 .12</size></box></geometry></collision>
        <visual name="body_visual"><geometry><box><size>.22 .16 .12</size></box></geometry>
          <material><ambient>.08 .25 .95 1</ambient><diffuse>.08 .25 .95 1</diffuse></material></visual>
        <sensor name="laser" type="gpu_lidar"><topic>/scan</topic><pose>0 0 .07 0 0 0</pose>
          <always_on>true</always_on><update_rate>10</update_rate><ray><scan><horizontal>
          <samples>360</samples><resolution>1</resolution><min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle></horizontal></scan><range><min>.08</min><max>6</max></range>
          </ray></sensor>
        <sensor name="camera" type="camera"><topic>/camera/image</topic>
          <pose>.115 0 .09 0 -.80 0</pose><always_on>true</always_on><update_rate>10</update_rate>
          <camera><horizontal_fov>1.15</horizontal_fov><image><width>640</width><height>480</height>
          <format>R8G8B8</format></image><clip><near>.05</near><far>6</far></clip></camera></sensor>
      </link>
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
        <axis><xyz>0 0 1</xyz></axis></joint>
      <joint name="right_wheel_joint" type="revolute"><parent>chassis</parent><child>right_wheel</child>
        <axis><xyz>0 0 1</xyz></axis></joint>
      <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
        <left_joint>left_wheel_joint</left_joint><right_joint>right_wheel_joint</right_joint>
        <wheel_separation>.20</wheel_separation><wheel_radius>.055</wheel_radius>
        <topic>/model/patrol_bot/cmd_vel</topic><odom_topic>/model/patrol_bot/odometry</odom_topic>
        <frame_id>odom</frame_id><child_frame_id>base_link</child_frame_id>
        <odom_publish_frequency>20</odom_publish_frequency>
      </plugin>
      <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher"/>
    </model>'''


def main():
    models = [
        box('ground', 0, 0, -.015, 4.2, 4.2, .03, colour(.24, .25, .26), collision=True),
        box('north_wall', 0, 2.09, .12, 4.2, .02, .24, colour(.55, .55, .55), collision=True),
        box('south_wall', 0, -2.09, .12, 4.2, .02, .24, colour(.55, .55, .55), collision=True),
        box('east_wall', 2.09, 0, .12, .02, 4.2, .24, colour(.55, .55, .55), collision=True),
        box('west_wall', -2.09, 0, .12, .02, 4.2, .24, colour(.55, .55, .55), collision=True),
    ]
    # White road marking strips; they are visual only so their collisions do not
    # interfere with the first-lap controller.
    for name, x, y, sx, sy in [
        ('top_lane_n', 0, 1.94, 4, .012), ('top_lane_s', 0, 1.16, 4, .012),
        ('left_lane_w', -1.94, .6, .012, 2.6),
        ('middle_lane_n', -.7, .86, 2.6, .012),
        ('middle_lane_s', -.7, .14, 2.6, .012),
        ('centre_lane_w', -.06, -.65, .012, 1.7),
        ('centre_lane_e', .66, -.65, .012, 1.7),
        ('bottom_lane_n', .9, -1.14, 2.2, .012),
        ('bottom_lane_s', .9, -1.94, 2.2, .012),
        ('right_lane_w', 1.14, 0, .012, 3.8),
        ('right_lane_e', 1.94, 0, .012, 3.8),
    ]:
        models.append(box(name, x, y, .002, sx, sy, .003, colour(.9, .9, .9)))
    # Stops before the two overhead light frames.
    models.extend([
        box('stop_line_top', -.18, 1.55, .004, .025, .65, .004, colour(1, 1, 1)),
        box('stop_line_bottom', .30, -1.04, .004, .65, .025, .004, colour(1, 1, 1)),
        traffic_light('light_top', -.57, 1.55, 0),
        traffic_light('light_bottom', .30, -1.39, 1.5707963),
    ])
    for i, (x, y) in enumerate([(-1.35, .96), (-1.15, .96), (-.95, .96), (-.75, .96), (-.55, .96)]):
        models.append(standee(f'A_resident_{i+1}', x, y))
    for i, (x, y) in enumerate([(-1.35, -.28), (-1.15, -.28), (-.95, -.28), (-.75, -.28)]):
        models.append(standee(f'B_resident_{i+1}', x, y))
    models.append(standee('B_outsider', -.55, -.28, outsider=True))
    for i, y in enumerate([-1.55, -.75, .05], 1):
        models.append(box(f'car_{i}', 1.77, y, .13, .35, .26, .25, colour(.05, .65, .75), collision=True))
        models.append(box(f'plate_{i}', 1.59, y, .13, .006, .095, .03, colour(.05, .05, .7)))
    models.append(robot())
    body = '\n'.join(models)
    sdf = f'''<?xml version="1.0"?>
<sdf version="1.9"><world name="community">
  <gravity>0 0 -9.8</gravity>
  <scene><ambient>.7 .7 .7 1</ambient><background_color>.4 .5 .6 1</background_color></scene>
  <light name="sun" type="directional"><pose>0 0 5 0 0 0</pose><diffuse>1 1 1 1</diffuse>
    <specular>.1 .1 .1 1</specular><direction>-.3 -.4 -1</direction></light>
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
  <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
  <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
    <render_engine>ogre2</render_engine></plugin>
  {body}
</world></sdf>'''
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(sdf, encoding='utf-8')
    print(OUT)


if __name__ == '__main__':
    main()
