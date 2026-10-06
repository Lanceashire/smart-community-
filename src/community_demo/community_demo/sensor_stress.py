"""Opt-in deterministic sensor perturbations for acceptance, never normal control."""
import copy
import json
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data,QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class SensorStress(Node):
    def __init__(self):
        super().__init__('sensor_stress_acceptance')
        self.declare_parameter('seed',20261004)
        self.seed=int(self.get_parameter('seed').value)
        self.rng=np.random.default_rng(self.seed)
        self.outputs={}
        latest=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.input_ages={k:[] for k in ['image','odometry','points']}
        self.counts={k:{'received':0,'published':0,'dropped':0,'blackout':0} for k in ['image','odometry','points']}
        self.blackout_start=None;self.blackout_end=None;self.blackout_samples=[]
        self.speed=0.;self.command=(0.,0.)
        for kind,topic,typ in [('image','/camera/image',Image),('odometry','/model/patrol_bot/odometry',Odometry),('points','/lidar/raw/points',PointCloud2)]:
            target={'image':'/stress/camera/image','odometry':'/stress/odometry','points':'/stress/points'}[kind]
            self.outputs[kind]=self.create_publisher(typ,target,latest)
            self.create_subscription(typ,topic,lambda m,k=kind:self.receive(k,m),latest)
        self.create_subscription(String,'/patrol/status',self.on_status,10)
        self.create_subscription(Odometry,'/model/patrol_bot/ground_truth',self.on_truth,qos_profile_sensor_data)
        self.create_subscription(Twist,'/model/patrol_bot/cmd_vel',lambda m:setattr(self,'command',(m.linear.x,m.angular.z)),10)
        self.statistics=self.create_publisher(String,'/stress/statistics',10)
        self.create_timer(.1,self.tick)

    def on_status(self,m):
        if self.blackout_start is None and m.data.startswith('GO top_west'):
            self.blackout_start=self.get_clock().now().nanoseconds/1e9
            self.blackout_end=self.blackout_start+2.

    def on_truth(self,m):
        self.speed=math.hypot(m.twist.twist.linear.x,m.twist.twist.linear.y)

    def receive(self,kind,m):
        c=self.counts[kind];c['received']+=1
        now=self.get_clock().now().nanoseconds/1e9
        stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9
        self.input_ages[kind].append(now-stamp)
        self.input_ages[kind]=self.input_ages[kind][-200:]
        if self.blackout_end is not None and now<self.blackout_end:
            c['blackout']+=1;return
        if self.rng.random()<.1:
            c['dropped']+=1;return
        m=copy.deepcopy(m)
        if kind=='image':
            pixels=np.frombuffer(m.data,np.uint8)
            noisy=np.clip(pixels.astype(np.float32)+self.rng.normal(0,3.,pixels.shape),0,255).astype(np.uint8)
            m.data=noisy.tobytes()
        elif kind=='odometry':
            m.pose.pose.position.x+=float(self.rng.normal(0,.001))
            m.pose.pose.position.y+=float(self.rng.normal(0,.001))
            q=m.pose.pose.orientation
            a=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))+float(self.rng.normal(0,.001))
            q.x=q.y=0.;q.z=math.sin(a/2);q.w=math.cos(a/2)
        else:
            fields={f.name:f.offset for f in m.fields}
            dtype=np.dtype({'names':['x','y','z'],'formats':['<f4']*3,'offsets':[fields[k] for k in 'xyz'],'itemsize':m.point_step})
            points=np.frombuffer(m.data,dtype=dtype).copy()
            for axis in 'xyz':points[axis]+=self.rng.normal(0,.005,len(points)).astype(np.float32)
            m.data=points.tobytes()
        self.outputs[kind].publish(m);c['published']+=1

    def tick(self):
        now=self.get_clock().now().nanoseconds/1e9
        if self.blackout_start is not None and self.blackout_start<=now<self.blackout_end:
            self.blackout_samples.append({'elapsed_s':round(now-self.blackout_start,3),'speed':self.speed,'command':self.command})
        late=[s for s in self.blackout_samples if s['elapsed_s']>=1.5]
        stopped=bool(late) and all(s['speed']<.015 and max(map(abs,s['command']))<.001 for s in late)
        self.statistics.publish(String(data=json.dumps({'seed':self.seed,'counts':self.counts,'input_age_p95_s':{k:float(np.percentile(v,95)) if v else None for k,v in self.input_ages.items()},
            'image_sigma_levels':3.,'odometry_position_sigma_m':.001,'odometry_yaw_sigma_rad':.001,
            'lidar_xyz_sigma_m':.005,'message_drop_probability':.1,
            'blackout_start_sim_s':self.blackout_start,'blackout_end_sim_s':self.blackout_end,
            'blackout_finished':self.blackout_end is not None and now>self.blackout_end,
            'blackout_stop_passed':stopped,'blackout_samples':self.blackout_samples})))


def main():
    rclpy.init();n=SensorStress()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.destroy_node();rclpy.try_shutdown()
