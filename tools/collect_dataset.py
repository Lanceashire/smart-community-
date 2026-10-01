#!/usr/bin/env python3
"""Project simulator geometry ONLY for offline annotation, never runtime answers.

Capture batches must be split by pose/recording, not adjacent video frames.
"""
import argparse,json,math,time,random,subprocess
from pathlib import Path
import cv2,numpy as np,rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Float64,String
from rclpy.qos import qos_profile_sensor_data
p=argparse.ArgumentParser();p.add_argument('--batch',default='lap01');p.add_argument('--seconds',type=float,default=210);p.add_argument('--random-views',type=int,default=0);p.add_argument('--split',default='train');p.add_argument('--head-views',action='store_true');p.add_argument('--settle',type=float,default=.65);p.add_argument('--layout',default='layout.json');p.add_argument('--seed',type=int,default=42);a=p.parse_args()
root=Path(__file__).resolve().parents[1];layout=json.loads((root/'src/community_demo/worlds'/a.layout).read_text());names=[f'resident_{i:02}' for i in range(1,17)]+['visitor_01','visitor_02','plate','light_red','light_yellow','light_green']
out=root/'data/community';(out/'images'/a.split).mkdir(parents=True,exist_ok=True);(out/'labels'/a.split).mkdir(parents=True,exist_ok=True)
(out/'dataset.yaml').write_text('path: '+str(out)+'\ntrain: images/train\nval: images/val\ntest: images/test\nnames:\n'+''.join(f'  {i}: {name}\n' for i,name in enumerate(names)))
rclpy.init();n=rclpy.create_node('offline_dataset_annotator');state={'pose':None,'pan':0.,'pan_at':0.,'phase':'red','frame':None,'frameid':0,'savedid':-1};counter=0;manifest=[]
def odom(m):
    q=m.pose.pose.orientation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));state['pose']=(m.pose.pose.position.x,m.pose.pose.position.y,yaw)
def pan(m):
    if abs(m.data-state['pan'])>.01:state.update(pan=m.data,pan_at=time.monotonic())
def frame(m):
    img=np.frombuffer(m.data,np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3)
    state['frame']=cv2.cvtColor(img,cv2.COLOR_RGB2BGR) if m.encoding=='rgb8' else img.copy();state['frameid']+=1
n.create_subscription(Odometry,'/model/patrol_bot/ground_truth',odom,10);n.create_subscription(Image,'/camera/image',frame,qos_profile_sensor_data);n.create_subscription(Float64,'/model/patrol_bot/camera_pan',pan,10);n.create_subscription(String,'/simulation/traffic_light_state',lambda m:state.update(phase=m.data),10)
panpub=n.create_publisher(Float64,'/model/patrol_bot/camera_pan',10)
def spin(seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.03)
def bbox(x,y,yaw,w,h,z,cam):
    corners=np.array([[x-math.sin(yaw)*u,y+math.cos(yaw)*u,z+v] for u in [-w/2,w/2] for v in [0,h]])
    angle=cam[2]+state['pan'];origin=np.array([cam[0]+.03*math.cos(angle),cam[1]+.03*math.sin(angle),.265]);d=corners-origin
    c,s=math.cos(angle),math.sin(angle);dx=c*d[:,0]+s*d[:,1];dy=-s*d[:,0]+c*d[:,1];cp,sp=math.cos(.1),math.sin(.1);depth=cp*dx-sp*d[:,2];up=sp*dx+cp*d[:,2]
    if min(depth)<.06:return None
    H,W=state['frame'].shape[:2];f=W/(2*math.tan(.8));u=W/2-f*dy/depth;v=H/2-f*up/depth;b=np.array([u.min(),v.min(),u.max(),v.max()]);clip=np.clip(b,[0,0,0,0],[W,H,W,H]);area=max(0,b[2]-b[0])*max(0,b[3]-b[1]);ca=max(0,clip[2]-clip[0])*max(0,clip[3]-clip[1])
    if area<120 or ca<area*.6 or clip[2]-clip[0]<7:return None
    return clip,float(depth.mean())
def save():
    global counter
    if state['frame'] is None or state['pose'] is None or state['frameid']==state['savedid'] or time.monotonic()-state['pan_at']<2.0:return
    cam=state['pose'];objects=[]
    for person in layout['people']:
        distance=math.hypot(cam[0]-person['x'],cam[1]-person['y']);dot=((cam[0]-person['x'])*math.cos(person['yaw'])+(cam[1]-person['y'])*math.sin(person['yaw']))/max(.01,distance)
        if dot<.32:continue
        b=bbox(person['x'],person['y'],person['yaw'],person['width_m'],.15,.004,cam)
        if b:objects.append((names.index(person['asset']),*b))
    for car in layout['cars']:
        if cam[0]>car['x']:continue
        b=bbox(car['x']-.0038,car['y'],math.pi,.095,.03,.044,cam)
        if b:objects.append((18,*b))
    for lamp in layout['lamps']:
        if lamp['state']!='on' or lamp['colour']!=state['phase']:continue
        if (cam[0]-lamp['x'])*math.cos(lamp['yaw'])+(cam[1]-lamp['y'])*math.sin(lamp['yaw'])<0:continue
        b=bbox(lamp['x'],lamp['y'],lamp['yaw'],.12,.12,.35,cam)
        if b:objects.append((names.index('light_'+lamp['colour']),*b))
    # Drop rear annotations occluded by a closer projected rectangle.
    objects.sort(key=lambda o:o[2]);kept=[]
    for obj in objects:
        _,b,_=obj;area=(b[2]-b[0])*(b[3]-b[1]);occluded=False
        for _,f,_ in kept:
            overlap=max(0,min(b[2],f[2])-max(b[0],f[0]))*max(0,min(b[3],f[3])-max(b[1],f[1]))
            if overlap/area>.25:occluded=True
        if not occluded:kept.append(obj)
    H,W=state['frame'].shape[:2];name=f'{a.batch}_{counter:05}';labels=[]
    for cls,b,depth in kept:labels.append(f'{cls} {(b[0]+b[2])/2/W:.6f} {(b[1]+b[3])/2/H:.6f} {(b[2]-b[0])/W:.6f} {(b[3]-b[1])/H:.6f}')
    cv2.imwrite(str(out/'images'/a.split/(name+'.jpg')),state['frame'],[cv2.IMWRITE_JPEG_QUALITY,94]);(out/'labels'/a.split/(name+'.txt')).write_text('\n'.join(labels)+'\n');manifest.append({'name':name,'pose':cam,'pan':state['pan'],'phase':state['phase'],'labels':len(labels)});counter+=1;state['savedid']=state['frameid']
    if counter%25==0:print('captured',counter,flush=True)
try:
    if a.random_views:
        if any(name=='/patrol/restart' for name,_ in n.get_service_names_and_types()):raise RuntimeError('Random views require scene-only launch. Never teleport an autonomous patrol.')
        random.seed(a.seed);anchors=[(-.74,1.7,-math.pi/2),(-1.7,1.04,0),(-.9,.5,math.pi/2),(-.9,.5,-math.pi/2),(-.1,-.09,math.pi),(1.1,-1.7,0),(1.1,-1.1,0),(1.1,-.5,0),(.52,1.7,math.pi),(-.1,-.29,-math.pi/2)]
        panpub.publish(Float64(data=0.));spin(3.)
        for i in range(a.random_views):
            anchor_index=(i//max(1,a.random_views//len(anchors)))%len(anchors) if a.head_views else i%len(anchors)
            x,y,yaw=anchors[anchor_index]
            if a.head_views:
                head=math.pi/2 if anchor_index<3 else (-math.pi/2 if anchor_index<8 else 0.)
                if abs(head-state['pan'])>.01:
                    panpub.publish(Float64(data=head));spin(3.)
                yaw-=head
            x+=random.uniform(-.13,.13);y+=random.uniform(-.13,.13);yaw+=random.uniform(-.18,.18)
            req=f'name:"patrol_bot" position{{x:{x} y:{y} z:.002}} orientation{{z:{math.sin(yaw/2)} w:{math.cos(yaw/2)}}}'
            for retry in range(4):
                r=subprocess.run(['gz','service','-s','/world/community/set_pose','--reqtype','gz.msgs.Pose','--reptype','gz.msgs.Boolean','--timeout','5000','--req',req],capture_output=True,text=True)
                if 'true' in r.stdout:break
                spin(.3)
            else:raise RuntimeError(r.stdout+r.stderr)
            spin(a.settle);save()
    else:
        end=time.monotonic()+a.seconds
        while time.monotonic()<end:spin(.35);save()
finally:
    (out/(a.batch+'-manifest.json')).write_text(json.dumps({'annotation':'offline projection from simulation, forbidden in runtime perception','seed':a.seed,'split':a.split,'frames':manifest},indent=2));print('saved',counter,flush=True);n.destroy_node();rclpy.try_shutdown()
