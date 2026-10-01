#!/usr/bin/env python3
"""Capture fresh, pose-confirmed, deduplicated validation/test camera frames.

Only offline annotations may use simulator truth. Never run with a patrol node.
Images remain local. Capture conditions and source poses are preserved per frame.
"""
import argparse
import hashlib
import json
import math
import random
import subprocess
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

ROOT = Path(__file__).resolve().parents[1]
NAMES = [f'resident_{i:02}' for i in range(1,17)] + ['visitor_01','visitor_02','plate','light_red','light_yellow','light_green']

def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec / 1e9

def service(name, reqtype, request):
    p = subprocess.run(['gz','service','-s',name,'--reqtype',reqtype,'--reptype','gz.msgs.Boolean',
        '--timeout','2500','--req',request], capture_output=True, text=True, timeout=4)
    if p.returncode or 'true' not in p.stdout:
        raise RuntimeError(p.stdout + p.stderr)

class Capture:
    def __init__(self, args):
        self.a = args
        self.layout = json.loads((ROOT/'src/community_demo/worlds'/args.layout).read_text())
        self.out = ROOT/'data/comparison'
        for kind in ['images','labels']:
            (self.out/kind/args.split).mkdir(parents=True, exist_ok=True)
        self.node = rclpy.create_node('offline_expanded_validation')
        self.image = None; self.pose = None; self.image_t = -1; self.pose_t = -1; self.image_seq = 0
        self.phase = 'red'; self.records = []; self.rejected = []
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.node.create_subscription(Image, '/camera/image', self.frame, qos)
        self.node.create_subscription(Odometry, '/model/patrol_bot/ground_truth', self.odom, qos)
        self.pan = self.node.create_publisher(Float64, '/model/patrol_bot/camera_pan', 10)
        self.hashes = set()
        for base in [ROOT/'data/community/images', self.out/'images']:
            for p in base.glob('*/*.jpg'):
                self.hashes.add(hashlib.sha256(p.read_bytes()).hexdigest())
        self.rng = random.Random(args.seed)

    def frame(self, m):
        arr = np.frombuffer(m.data,np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3)
        self.image = cv2.cvtColor(arr,cv2.COLOR_RGB2BGR) if m.encoding == 'rgb8' else arr.copy()
        self.image_t = stamp(m); self.image_seq += 1

    def odom(self,m):
        q=m.pose.pose.orientation;p=m.pose.pose.position
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        self.pose=(p.x,p.y,yaw);self.pose_t=stamp(m)

    def spin(self,seconds):
        until=time.monotonic()+seconds
        while time.monotonic()<until:rclpy.spin_once(self.node,timeout_sec=.02)

    def phase_to(self, colour):
        poses=[]
        for lamp in self.layout['lamps']:
            visible=(lamp['state']=='on')==(lamp['colour']==colour)
            z=lamp['visible_z'] if visible else lamp['hidden_z'];yaw=lamp['yaw']
            poses.append(f'pose {{ name: "{lamp["name"]}" position {{ x: {lamp["x"]} y: {lamp["y"]} z: {z} }} orientation {{ z: {math.sin(yaw/2)} w: {math.cos(yaw/2)} }} }}')
        service('/world/community/set_pose_vector','gz.msgs.Pose_V',' '.join(poses))
        self.phase=colour;self.spin(.4)

    def move(self,x,y,yaw):
        self.spin(.05)
        old_t=self.pose_t;seq=self.image_seq
        service('/world/community/set_pose','gz.msgs.Pose',f'name: "patrol_bot" position {{ x: {x} y: {y} z: .002 }} orientation {{ z: {math.sin(yaw/2)} w: {math.cos(yaw/2)} }}')
        until=time.monotonic()+4;confirmed_t=None
        while time.monotonic()<until:
            self.spin(.02)
            if self.pose is None:continue
            dx=math.hypot(self.pose[0]-x,self.pose[1]-y)
            dyaw=abs(math.atan2(math.sin(self.pose[2]-yaw),math.cos(self.pose[2]-yaw)))
            if confirmed_t is None and self.pose_t>old_t and dx<.035 and dyaw<.07:
                confirmed_t=self.pose_t
            if (confirmed_t is not None and dx<.035 and dyaw<.07 and self.image_seq>=seq+3
                    and self.image_t>confirmed_t+.12 and abs(self.image_t-self.pose_t)<.075):
                return True
        self.rejected.append({'reason':'pose_or_frame_timeout','requested':[x,y,yaw],'actual':self.pose})
        return False

    def bbox(self,x,y,yaw,w,h,z):
        corners=np.array([[x-math.sin(yaw)*u,y+math.cos(yaw)*u,z+v] for u in [-w/2,w/2] for v in [0,h]])
        # Offline static base is at z=.002; camera link is .265 above its origin.
        cam=self.pose;angle=cam[2];origin=np.array([cam[0]+.03*math.cos(angle),cam[1]+.03*math.sin(angle),.267]);d=corners-origin
        c,s=math.cos(angle),math.sin(angle);dx=c*d[:,0]+s*d[:,1];dy=-s*d[:,0]+c*d[:,1]
        cp,sp=math.cos(.1),math.sin(.1);depth=cp*dx-sp*d[:,2];up=sp*dx+cp*d[:,2]
        if min(depth)<.06:return None
        H,W=self.image.shape[:2];f=W/(2*math.tan(.8));u=W/2-f*dy/depth;v=H/2-f*up/depth
        b=np.array([u.min(),v.min(),u.max(),v.max()]);clip=np.clip(b,[0,0,0,0],[W,H,W,H])
        area=max(0,b[2]-b[0])*max(0,b[3]-b[1]);ca=max(0,clip[2]-clip[0])*max(0,clip[3]-clip[1])
        if area<120 or ca<area*.7 or clip[2]-clip[0]<7:return None
        return clip,float(depth.mean())

    def labels(self):
        objects=[];cam=self.pose
        for p in self.layout['people']:
            dist=math.hypot(cam[0]-p['x'],cam[1]-p['y'])
            dot=((cam[0]-p['x'])*math.cos(p['yaw'])+(cam[1]-p['y'])*math.sin(p['yaw']))/max(.01,dist)
            if dot<.35:continue
            b=self.bbox(p['x'],p['y'],p['yaw'],p['width_m'],.15,.004)
            if b:objects.append((NAMES.index(p['asset']),*b))
        for p in self.layout['cars']:
            if cam[0]>p['x']:continue
            b=self.bbox(p['x']-.0038,p['y'],math.pi,.095,.03,.044)
            if b:objects.append((18,*b))
        for p in self.layout['lamps']:
            if p['state']!='on' or p['colour']!=self.phase:continue
            if (cam[0]-p['x'])*math.cos(p['yaw'])+(cam[1]-p['y'])*math.sin(p['yaw'])<0:continue
            b=self.bbox(p['x'],p['y'],p['yaw'],.12,.12,.35)
            if b:objects.append((NAMES.index('light_'+p['colour']),*b))
        kept=[]
        for obj in sorted(objects,key=lambda x:x[2]):
            _,b,_=obj;area=(b[2]-b[0])*(b[3]-b[1])
            if any(max(0,min(b[2],o[1][2])-max(b[0],o[1][0]))*max(0,min(b[3],o[1][3])-max(b[1],o[1][1]))/area>.2 for o in kept):continue
            kept.append(obj)
        H,W=self.image.shape[:2]
        return [f'{cls} {(b[0]+b[2])/2/W:.6f} {(b[1]+b[3])/2/H:.6f} {(b[2]-b[0])/W:.6f} {(b[3]-b[1])/H:.6f}' for cls,b,_ in kept]

    def candidate(self,index):
        targets=self.layout['people']+self.layout['cars']+[p for p in self.layout['lamps'] if p['state']=='on' and p['colour']==self.phase]
        p=targets[index%len(targets)];angle=p['yaw']+self.rng.uniform(-.75,.75)
        distance=self.rng.uniform(.38,.95) if 'asset' in p else self.rng.uniform(.35,.9)
        x=max(-2.03,min(2.03,p['x']+distance*math.cos(angle)))
        y=max(-2.03,min(2.03,p['y']+distance*math.sin(angle)))
        yaw=math.atan2(p['y']-y,p['x']-x)+self.rng.uniform(-.22,.22)
        return x,y,yaw

    def run(self):
        self.spin(2)
        if any(name=='/patrol/restart' for name,_ in self.node.get_service_names_and_types()):
            raise RuntimeError('Capture requires isolated validation_scene.launch.py')
        self.pan.publish(Float64(data=0.));self.spin(1)
        attempt=0;wanted=self.a.count
        while len(self.records)<wanted and attempt<wanted*4:
            colour=['red','green','yellow'][(len(self.records)//max(1,wanted//3))%3]
            if colour!=self.phase or attempt==0:self.phase_to(colour)
            requested=self.candidate(attempt);attempt+=1
            if not self.move(*requested):continue
            ok,encoded=cv2.imencode('.jpg',self.image,[cv2.IMWRITE_JPEG_QUALITY,94])
            if not ok:continue
            digest=hashlib.sha256(encoded.tobytes()).hexdigest()
            if digest in self.hashes:
                self.rejected.append({'reason':'exact_duplicate'});continue
            labels=self.labels()
            if not labels:
                self.rejected.append({'reason':'no_visible_label'});continue
            name=f'{self.a.batch}_{len(self.records):05d}'
            (self.out/'images'/self.a.split/(name+'.jpg')).write_bytes(encoded.tobytes())
            (self.out/'labels'/self.a.split/(name+'.txt')).write_text('\n'.join(labels)+'\n')
            self.hashes.add(digest)
            self.records.append({'name':name,'sha256':digest,'pose':self.pose,'requested_pose':requested,'image_stamp':self.image_t,'pose_stamp':self.pose_t,'phase':self.phase,'classes':[int(s.split()[0]) for s in labels]})
            if len(self.records)%20==0:print(self.a.batch,'saved',len(self.records),flush=True)
        self.write_manifest()
        print('CAPTURE_DONE',self.a.batch,len(self.records),'rejected',len(self.rejected),flush=True)
        if len(self.records)<wanted:raise RuntimeError('Capture quota unmet')

    def write_manifest(self):
        report={'split':self.a.split,'batch':self.a.batch,'condition':self.a.condition,'layout':self.a.layout,'seed':self.a.seed,'capture_rig':'Static collision-free offline rig, same robot camera intrinsics/extrinsics; not navigation evidence','annotation':'Offline geometric projection; automatically filtered occlusion; not exhaustive manual labeling','frames':self.records,'rejections':self.rejected}
        (self.out/(self.a.batch+'-manifest.json')).write_text(json.dumps(report,indent=2))

def main():
    p=argparse.ArgumentParser();p.add_argument('--batch',required=True);p.add_argument('--split',choices=['val','test'],required=True);p.add_argument('--count',type=int,required=True);p.add_argument('--seed',type=int,required=True);p.add_argument('--condition',required=True);p.add_argument('--layout',default='layout.json');a=p.parse_args()
    rclpy.init();capture=Capture(a)
    try:capture.run()
    finally:capture.write_manifest();capture.node.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':main()
