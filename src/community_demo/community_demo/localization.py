"""Camera lane registration + incremental wheel odometry + source-time IMU heading.

Only this evaluator's external tools access simulator truth. The runtime node
matches observed ground paint to the supplied map, using the known start pose.
"""
import math
import json
from collections import deque
from pathlib import Path
import cv2
from community_demo.paint_features import white_paint_mask
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data,QoSProfile,ReliabilityPolicy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, LaserScan, Image, JointState, Imu
from community_demo.imu_heading import imu_sample,heading_at
from std_msgs.msg import Float64,String
from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster
from community_demo.patrol import yaw_from_quaternion,wrap
from community_demo.pose_sync import interpolate,field_pose
from community_demo.sensor_validation import valid_rgb_image,valid_odometry,pan_sample,stamp_ns,decode_xyz_cloud

class Localization(Node):
    def __init__(self):
        super().__init__('community_localization')
        cv2.setNumThreads(2)
        self.tf=TransformBroadcaster(self);self.static=StaticTransformBroadcaster(self)
        self.poses=self.create_publisher(Odometry,'/localization/pose',qos_profile_sensor_data)
        self.scans=self.create_publisher(LaserScan,'/scan',qos_profile_sensor_data)
        self.cloud_source_ns=None
        self.status=self.create_publisher(String,'/localization/status',10)
        self.xy=np.array([1.1,1.7]);self.yaw=math.pi;self.previous=None;self.pan=0.;self.pan_time=-1e9;self.frame_time=-1e9;self.odom_time=None;self.odom_received=-1e9;self.camera_received=-1e9
        source=Path(get_package_share_directory('community_demo'))/'models/community_assets/materials/textures/floor.png'
        floor=cv2.imread(str(source),0);self.distance=cv2.distanceTransform((floor<210).astype(np.uint8),cv2.DIST_L2,5)*4.2/floor.shape[0]
        self.map_n=floor.shape[0];self.score=None
        self.pan_samples=deque(maxlen=512);self.encoder_age=None
        self.last_match=None;self.unmatched_distance=0.;self.registration_points=0
        self.wheel_samples=deque(maxlen=256);self.pending_image=None
        self.registration_stamp=None;self.image_odom_delta=None;self.registration_corrections=0
        self.axis_improvements=None
        self.camera_source_time=None
        self.require_imu=True;self.imu_samples=deque(maxlen=512);self.imu_reference=None
        self.create_subscription(Imu,'/imu/data',self.imu,qos_profile_sensor_data)
        self.quality=self.create_publisher(String,'/localization/quality',qos_profile_sensor_data)
        grid=np.array(np.meshgrid(np.linspace(-.045,.045,7),np.linspace(-.045,.045,7),np.linspace(-.04,.04,7),indexing='ij')).reshape(3,-1).T
        self.grid=grid
        latest=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry,'/model/patrol_bot/odometry',self.odom,latest)
        self.create_subscription(PointCloud2,'/lidar/raw/points',self.cloud,latest)
        self.create_subscription(Image,'/camera/image',self.queue_image,latest)
        self.create_subscription(Float64,'/model/patrol_bot/camera_pan',self.on_pan,qos_profile_sensor_data)
        self.create_subscription(JointState,'/world/community/model/patrol_bot/joint_state',self.on_joint_state,qos_profile_sensor_data)
        static=[]
        for parent,child,xyz,yaw in [('field','slam_map',(1.1,1.7,0),math.pi),('slam_base','laser_frame',(0,0,.235),0)]:
            t=TransformStamped();t.header.stamp=self.get_clock().now().to_msg();t.header.frame_id=parent;t.child_frame_id=child
            t.transform.translation.x,t.transform.translation.y,t.transform.translation.z=map(float,xyz)
            t.transform.rotation.z=math.sin(yaw/2);t.transform.rotation.w=math.cos(yaw/2);static.append(t)
        self.static.sendTransform(static);self.create_timer(.05,self.publish)
    def imu(self,m):
        sample=imu_sample(m)
        if sample is None:return
        t,yaw,rate=sample;age=self.get_clock().now().nanoseconds/1e9-t
        if not 0<=age<=.15 or (self.imu_samples and t<=self.imu_samples[-1][0]):return
        if self.imu_reference is None:self.imu_reference=yaw
        self.imu_samples.append((t,wrap(math.pi+yaw-self.imu_reference),rate))

    def on_joint_state(self,m):
        sample=pan_sample(m)
        if sample is None:return
        if self.pan_samples and sample[0]<=self.pan_samples[-1][0]:return
        self.pan_samples.append(sample)

    def on_pan(self,m):
        if not math.isfinite(m.data):return
        if abs(m.data-self.pan)>.01:self.pan=m.data;self.pan_time=self.get_clock().now().nanoseconds/1e9
    def odom(self,m):
        if not valid_odometry(m):return
        age_ns=self.get_clock().now().nanoseconds-stamp_ns(m.header.stamp)
        if age_ns < -50_000_000 or age_ns > 150_000_000:return
        stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9
        if self.wheel_samples and stamp<=self.wheel_samples[-1][0]:return
        self.odom_received=self.get_clock().now().nanoseconds/1e9
        p=m.pose.pose.position;q=yaw_from_quaternion(m.pose.pose.orientation);current=np.array([p.x,p.y,q])
        if self.previous is not None:
            delta=current[:2]-self.previous[:2];angle=self.yaw-self.previous[2]
            self.unmatched_distance+=float(np.linalg.norm(delta))
            c,s=math.cos(angle),math.sin(angle)
            self.xy+=np.array([c*delta[0]-s*delta[1],s*delta[0]+c*delta[1]])
            self.yaw=wrap(self.yaw+wrap(q-self.previous[2]))
        self.previous=current;self.odom_time=m.header.stamp
        self.wheel_samples.append((stamp,tuple(current)))
        imu_yaw=heading_at(getattr(self,'imu_samples',[]),stamp)
        if imu_yaw is not None:self.yaw=imu_yaw
    def queue_image(self,m):
        if not valid_rgb_image(m):return
        now_ns=self.get_clock().now().nanoseconds
        now=now_ns/1e9
        stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9
        age_ns=now_ns-stamp_ns(m.header.stamp)
        if age_ns < -50_000_000 or age_ns > 700_000_000:return
        if self.camera_source_time is not None and stamp<=self.camera_source_time:return
        self.camera_source_time=stamp;self.camera_received=now
        self.pending_image=m
    def image(self,m):
        now=self.get_clock().now().nanoseconds/1e9
        if now-self.frame_time<.20-1e-9:return
        image_time=m.header.stamp.sec+m.header.stamp.nanosec/1e9
        if not valid_rgb_image(m):return
        image_age_ns=self.get_clock().now().nanoseconds-stamp_ns(m.header.stamp)
        if not 0<=image_age_ns<=200_000_000:return
        wheel=interpolate(self.wheel_samples,image_time)
        if wheel is None or self.previous is None:return
        image_pose=field_pose((*self.xy,self.yaw),self.previous,wheel)
        image_imu_yaw=heading_at(getattr(self,'imu_samples',[]),image_time)
        if getattr(self,'require_imu',False) and image_imu_yaw is None:return
        if image_imu_yaw is not None:image_pose=(*image_pose[:2],image_imu_yaw)
        self.image_odom_delta=self.wheel_samples[-1][0]-image_time
        if not self.pan_samples:return
        encoder_time,measured_pan,pan_velocity=min(self.pan_samples,key=lambda x:abs(x[0]-image_time))
        self.encoder_age=abs(encoder_time-image_time)
        # Side-looking views contain vertical portrait/plate paint, not just ground.
        # Do not feed moving or side-facing camera evidence into floor registration.
        if self.encoder_age>.10 or abs(measured_pan)>.06 or abs(pan_velocity)>.15:return
        self.frame_time=now
        a=np.frombuffer(m.data,np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3)
        if m.encoding=='rgb8':a=cv2.cvtColor(a,cv2.COLOR_RGB2BGR)
        # Below the horizon and above the robot hood; white ground paint only.
        mask,self.paint_threshold,self.paint_contrast=white_paint_mask(a)
        v,u=np.where(mask)
        self.registration_points=len(u)
        if len(u)<50:return
        indices=np.linspace(0,len(u)-1,min(400,len(u))).astype(int);u=u[indices];v=v[indices]
        f=m.width/(2*math.tan(.8));ry=-(u-m.width/2)/f;rz=-(v-m.height/2)/f
        cp,sp=math.cos(.10),math.sin(.10);dx=cp+sp*rz;dz=-sp+cp*rz;t=-.265/dz
        px=t*dx;py=t*ry;valid=(t>0)&(px<2.5);px=px[valid];py=py[valid]
        if len(px)<40:return
        c,s=math.cos(measured_pan),math.sin(measured_pan);bx=.03*c+c*px-s*py;by=.03*s+s*px+c*py
        def costs(grid):
            angles=image_pose[2]+grid[:,2];c=np.cos(angles)[:,None];s=np.sin(angles)[:,None]
            x=image_pose[0]+grid[:,0,None]+c*bx-s*by;y=image_pose[1]+grid[:,1,None]+s*bx+c*by
            ix=np.clip(((x+2.1)/4.2*(self.map_n-1)).astype(int),0,self.map_n-1);iy=np.clip(((2.1-y)/4.2*(self.map_n-1)).astype(int),0,self.map_n-1)
            distances=self.distance[iy,ix];cut=max(20,int(distances.shape[1]*.7));trim=np.partition(distances,cut-1,axis=1)[:,:cut]
            return np.mean(trim,axis=1)+.03*np.hypot(grid[:,0],grid[:,1])+.015*np.abs(grid[:,2])
        grid=self.grid.copy()
        if image_imu_yaw is not None:grid[:,2]=0
        scores=costs(grid);best=grid[np.argmin(scores)];fine=np.array(np.meshgrid(np.linspace(-.009,.009,5),np.linspace(-.009,.009,5),np.linspace(-.008,.008,5),indexing='ij')).reshape(3,-1).T+best
        if image_imu_yaw is not None:fine[:,2]=0
        scores=costs(fine);delta=fine[np.argmin(scores)];score=float(scores.min());self.score=score
        if score<.025:
            self.last_match=now;self.registration_stamp=image_time;self.unmatched_distance=0.
            # Reject needless corrections on ambiguous straight paint; cap each accepted update.
            if float(costs(np.zeros((1,3)))[0])-score>.0002:
                # Parallel lane paint cannot constrain travel along its length.
                # Test each coordinate against the same optimum before changing it.
                alternatives=np.tile(delta,(3,1));np.fill_diagonal(alternatives,0.)
                self.axis_improvements=(costs(alternatives)-score).tolist()
                accepted=delta*np.array([v>limit for v,limit in zip(self.axis_improvements,(.0005,.0005,.0002))])
                corrected=(image_pose[0]+float(np.clip(accepted[0]*.65,-.025,.025)),
                           image_pose[1]+float(np.clip(accepted[1]*.65,-.025,.025)),
                           wrap(image_pose[2]+float(np.clip(accepted[2]*.35,-.012,.012))))
                current=field_pose(corrected,wheel,self.previous)
                current_imu_yaw=heading_at(getattr(self,'imu_samples',[]),self.wheel_samples[-1][0])
                if current_imu_yaw is not None:current=(*current[:2],current_imu_yaw)
                self.xy=np.array(current[:2]);self.yaw=current[2];self.registration_corrections+=1
    def cloud(self,m):
        source_ns=stamp_ns(m.header.stamp)
        if source_ns is None or not 0 <= self.get_clock().now().nanoseconds-source_ns <= 500_000_000:
            return
        if self.cloud_source_ns is not None and source_ns <= self.cloud_source_ns:
            return
        xyz=decode_xyz_cloud(m)
        if xyz is None:return
        self.cloud_source_ns=source_ns
        x,y,z=xyz.T;r=np.hypot(x,y)
        valid=np.isfinite(r)&np.isfinite(z)&(r>.17)&(z>-.210)&(z<.10);a=np.arctan2(y[valid],x[valid]);r=r[valid];bins=720
        indices=np.clip(np.floor((a+math.pi)/(2*math.pi)*bins).astype(int),0,bins-1);ranges=np.full(bins,np.inf,np.float32);np.minimum.at(ranges,indices,np.where(r<=6.,r,np.inf))
        scan=LaserScan();scan.header=m.header;scan.header.frame_id='laser_frame';scan.angle_min=-math.pi;scan.angle_increment=2*math.pi/bins;scan.angle_max=scan.angle_min+(bins-1)*scan.angle_increment
        scan.range_min=.17;scan.range_max=6.;scan.scan_time=.1;scan.ranges=ranges.tolist();self.scans.publish(scan)
    def publish(self):
        now=self.get_clock().now().nanoseconds/1e9
        if self.pending_image is not None and self.wheel_samples:
            image=self.pending_image;t=image.header.stamp.sec+image.header.stamp.nanosec/1e9
            if t<=now and t<=self.wheel_samples[-1][0]:
                self.pending_image=None;self.image(image)
            elif now-t>.20:self.pending_image=None
        now_ns=self.get_clock().now().nanoseconds
        if getattr(self,'require_imu',False) and (not self.imu_samples or not 0<=now-self.imu_samples[-1][0]<=.15):
            self.quality.publish(String(data=json.dumps({'state':'lost','reason':'IMU source freshness'})))
            self.status.publish(String(data='LOST IMU; navigation pose withheld'));return
        odom_age_ns=None if self.odom_time is None else now_ns-(self.odom_time.sec*1_000_000_000+self.odom_time.nanosec)
        odom_age=None if odom_age_ns is None else odom_age_ns/1e9
        if odom_age_ns is None or odom_age_ns>150_000_000 or odom_age_ns< -50_000_000 or now-self.odom_received>.5 or now-self.camera_received>.7 or self.camera_source_time is None or now-self.camera_source_time>.7:
            self.quality.publish(String(data=json.dumps({'state':'lost','reason':'source or receipt freshness','odometry_source_age_s':odom_age,'odometry_receipt_age_s':now-self.odom_received,'camera_receipt_age_s':now-self.camera_received,'camera_source_age_s':None if self.camera_source_time is None else now-self.camera_source_time})))
            self.status.publish(String(data='LOST odometry or camera; navigation pose withheld'));return
        state='lost' if self.unmatched_distance>.75 else 'degraded' if self.unmatched_distance>.30 else 'tracking'
        self.quality.publish(String(data=json.dumps({'state':state,'odometry_source_age_s':round(odom_age,4),'encoder_image_delta_s':self.encoder_age,'image_odom_delta_s':self.image_odom_delta,'registration_stamp':self.registration_stamp,'registration_corrections':self.registration_corrections,'axis_improvements_m':self.axis_improvements,'score_m':self.score,'paint_pixels':self.registration_points,'paint_value_threshold':getattr(self,'paint_threshold',None),'paint_contrast':getattr(self,'paint_contrast',None),'distance_without_match_m':round(self.unmatched_distance,4),'last_match_age_s':None if self.last_match is None else round(now-self.last_match,3),'imu_source_age_s':None if not getattr(self,'imu_samples',[]) else round(now-self.imu_samples[-1][0],4),'criterion':'Image-time wheel interpolation, IMU heading and XY correction propagated forward; not a calibrated probability or global relocalization'})))
        # Pose belongs to the last integrated wheel sample, not the publication time.
        stamp=self.odom_time;transforms=[]
        # Separate mapping base avoids assigning two parents to navigation base.
        for parent,child,x,y,yaw in [('field','base_link',*self.xy,self.yaw),('odom','slam_base',1.1-self.xy[0],1.7-self.xy[1],wrap(self.yaw-math.pi))]:
            t=TransformStamped();t.header.stamp=stamp;t.header.frame_id=parent;t.child_frame_id=child;t.transform.translation.x=float(x);t.transform.translation.y=float(y);t.transform.rotation.z=math.sin(yaw/2);t.transform.rotation.w=math.cos(yaw/2);transforms.append(t)
        self.tf.sendTransform(transforms)
        m=Odometry();m.header.stamp=stamp;m.header.frame_id='field';m.child_frame_id='base_link';m.pose.pose.position.x=float(self.xy[0]);m.pose.pose.position.y=float(self.xy[1]);m.pose.pose.orientation=transforms[0].transform.rotation;self.poses.publish(m)
        self.status.publish(String(data=f'camera-wheel-imu; paint_score_m={self.score}'))

def main():
    rclpy.init();node=Localization()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()
