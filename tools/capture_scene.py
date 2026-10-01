"""Scene-only camera checks: teleports for inspection, never lap evidence.

Run with scene.launch.py (patrol disabled). Restores start pose afterward.
"""
import json
import math
from pathlib import Path
import subprocess
import time
import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from nav_msgs.msg import Odometry


def main():
    rclpy.init();node=rclpy.create_node('scene_camera_inspector')
    if any(n=='/patrol/restart' for n,_ in node.get_service_names_and_types()):
        raise RuntimeError('Use scene.launch.py: inspection may not teleport during an autonomous lap.')
    output=Path('log/scene-camera');output.mkdir(parents=True,exist_ok=True)
    state={'frame':None,'count':0,'signal':None,'pose':None,'vision':None}
    def frame(m):
        a=np.frombuffer(m.data,np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3)
        state['frame']=cv2.cvtColor(a,cv2.COLOR_RGB2BGR) if m.encoding=='rgb8' else a.copy();state['count']+=1
    def pose(m):
        p=m.pose.pose.position;state['pose']=[p.x,p.y,p.z]
    node.create_subscription(Image,'/camera/image',frame,qos_profile_sensor_data)
    node.create_subscription(String,'/simulation/traffic_light_state',lambda m:state.update(signal=m.data),10)
    node.create_subscription(String,'/perception/traffic_light',lambda m:state.update(vision=m.data),10)
    node.create_subscription(Odometry,'/model/patrol_bot/ground_truth',pose,10)
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
    def move(x,y,yaw):
        request=f'name: "patrol_bot" position {{ x: {x} y: {y} z: .002 }} orientation {{ z: {math.sin(yaw/2)} w: {math.cos(yaw/2)} }}'
        result=subprocess.run(['gz','service','-s','/world/community/set_pose','--reqtype','gz.msgs.Pose','--reptype','gz.msgs.Boolean','--timeout','2000','--req',request],capture_output=True,text=True,timeout=3)
        if result.returncode or 'true' not in result.stdout:raise RuntimeError(result.stderr+result.stdout)
        spin(1.0)
    evidence=[]
    def save(name):
        if state['frame'] is None:raise RuntimeError('No camera frame')
        cv2.imwrite(str(output/(name+'.png')),state['frame'])
        evidence.append({'view':name,'size':list(state['frame'].shape[:2][::-1]),'actual_robot_xyz':state['pose'],'signal_simulator':state['signal'],'camera_colour_baseline':state['vision']})
        print(name,evidence[-1],flush=True)
    try:
        for name,x,y,yaw in [('A_from_middle',-.87,.50,math.pi/2),('A_from_west',-1.7,1.10,0),
                              ('B_from_middle',-.87,.50,-math.pi/2),('B_from_east',-.1,-.20,math.pi),
                              ('plate_1',1.1,-1.7,0),('plate_2',1.1,-1.1,0),('plate_3',1.1,-.5,0)]:
            move(x,y,yaw);save(name)
        move(.52,1.7,math.pi)
        seen=set();end=time.monotonic()+45
        while len(seen)<3 and time.monotonic()<end:
            spin(.5);colour=state['signal']
            if colour in ['red','green','yellow'] and colour not in seen:
                spin(.5)
                if state['signal']==colour:save('signal_'+colour);seen.add(colour)
        if len(seen)!=3:raise RuntimeError('Did not observe all three phases in camera inspection')
    finally:
        move(1.1,1.7,math.pi);node.destroy_node();rclpy.shutdown()
    report={'mode':'Scene-only inspection; pose resets are NOT autonomous navigation evidence','views':evidence}
    (output/'evidence.json').write_text(json.dumps(report,indent=2)+'\n')


if __name__=='__main__':main()
