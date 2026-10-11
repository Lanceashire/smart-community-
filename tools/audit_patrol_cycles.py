#!/usr/bin/env python3
"""Live repeated-lap audit. Ground truth is read only; faults only pause named nodes."""
import bisect
import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

os.environ.setdefault("FASTDDS_BUILTIN_TRANSPORTS","UDPv4")

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan,CompressedImage
from std_msgs.msg import String
from std_srvs.srv import Trigger
from community_demo.road_rules import footprint, in_road

ROOT=Path(__file__).resolve().parents[1]
from community_demo.task_geometry import START,in_start_bay
from community_demo.road_rules import body_corners

EXPECTED_ROUTE=['top_approach','top_stop','observe_A_north','top_west','observe_A_west','left_south','observe_middle','middle_east','observe_B_east','lower_approach','lower_stop','lower_south','bottom_east','observe_plate_1','observe_plate_2','observe_plate_3','right_north']
EXPECTED_VIEWS=['A_north','A_west','A_south','B_north','B_east','plate_1','plate_2','plate_3']
EXPECTED_PLATES={'苏AB8Q62','鄂D7B5Q2','苏APL12A'}

def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def wrap(a):return math.atan2(math.sin(a),math.cos(a))
def stamp(h):return h.stamp.sec+h.stamp.nanosec*1e-9

def process_for(node):
    needle={'perception':'-m community_demo.perception','localization':'/community_demo/localization','patrol':'/community_demo/patrol'}[node]
    matches=[]
    for p in Path('/proc').iterdir():
        if not p.name.isdigit():continue
        try:cmd=(p/'cmdline').read_bytes().replace(b'\0',b' ').decode()
        except (OSError,UnicodeError):continue
        if needle in cmd and 'python' in cmd:matches.append(int(p.name))
    if len(matches)!=1:raise RuntimeError(f'Expected one {node} process, got {matches}')
    return matches[0]

def pose_at_source(samples,times,ns):
    if ns in samples:return samples[ns]
    i=bisect.bisect_left(times,ns)
    if i==0 or i==len(times):return None
    ta,tb=times[i-1:i+1]
    if tb-ta>150_000_000:return None
    a,b=samples[ta],samples[tb];k=(ns-ta)/(tb-ta)
    return a[0]+k*(b[0]-a[0]),a[1]+k*(b[1]-a[1]),wrap(a[2]+k*wrap(b[2]-a[2]))

def align_signal_evidence(records,samples):
    phase_times=sorted(samples);ages=[]
    for r in records:
        ns=round(r['t']*1e9);i=bisect.bisect_left(phase_times,ns)
        candidates=phase_times[max(0,i-1):i+1]
        nearest=min(candidates,key=lambda t:abs(t-ns)) if candidates else None
        delta=None if nearest is None else abs(nearest-ns)/1e9
        r.setdefault('phase_at_receipt',r['phase']);r['phase_pose_time_delta_s']=delta
        r['phase']=samples[nearest] if delta is not None and delta<=.075 else 'unknown'
        ages.append(delta)
    return ages

class Audit(Node):
    def __init__(self,args):
        super().__init__('patrol_cycle_audit')
        self.args=args;self.current=None;self.key=None;self.status='';self.phase='unknown';self.phase_samples={};self.estimate_samples={};self.snapshot_freeze=False;self.vision=None
        self.pose=None;self.estimate=None;self.command=(0.,0.);self.camera_stamps=[];self.image_stamps=set();self.scans=0
        self.event_checks=[];self.paused={};self.active_fault=None;self.faults=[];self.record_start=time.monotonic()
        self.first_counts=None;self.complete_at=None;self.complete_events=None;self.restart_rejected=None
        self.signal_diagnostics=[];self.signal_decisions=[];self.stress_statistics=[]
        self.localization_quality=[];self.approach_decisions=[];self.signal_holds=[]
        self.restart_client=self.create_client(Trigger,'/patrol/restart')
        self.create_subscription(String,'/patrol/lap',self.lap,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String,'/patrol/status',self.on_status,50)
        self.create_subscription(String,'/simulation/traffic_light_stamped',self.stamped_phase,20)
        self.create_subscription(String,'/simulation/traffic_light_state',lambda m:setattr(self,'phase',m.data),20)
        self.create_subscription(String,'/perception/results',self.perception,20)
        self.create_subscription(String,'/inspection/observation',self.observation,20)
        self.create_subscription(String,'/simulation/traffic_light_diagnostics',lambda m:self.signal_diagnostics.append(json.loads(m.data)),20)
        self.create_subscription(String,'/patrol/signal_hold',lambda m:self.signal_holds.append(json.loads(m.data)),50)
        self.create_subscription(String,'/patrol/signal_decision',lambda m:self.signal_decisions.append(json.loads(m.data)),20)
        self.create_subscription(String,'/stress/statistics',lambda m:self.stress_statistics.append(json.loads(m.data)),20)
        self.create_subscription(String,'/localization/quality',lambda m:self.localization_quality.append(json.loads(m.data)),qos_profile_sensor_data)
        self.create_subscription(String,'/patrol/approach_decision',lambda m:self.approach_decisions.append(json.loads(m.data)),20)
        self.create_subscription(Odometry,'/model/patrol_bot/ground_truth',self.truth,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/localization/pose',self.localization,qos_profile_sensor_data)
        self.create_subscription(Twist,'/model/patrol_bot/cmd_vel',lambda m:setattr(self,'command',(m.linear.x,m.angular.z)),20)
        self.create_subscription(Image,'/camera/image',lambda m:self.camera_stamps.append(stamp(m.header)),qos_profile_sensor_data)
        self.create_subscription(CompressedImage,'/perception/image/compressed',lambda m:self.image_stamps.add(round(stamp(m.header),6)),qos_profile_sensor_data)
        self.create_subscription(Image,'/perception/image',lambda m:self.image_stamps.add(round(stamp(m.header),6)),qos_profile_sensor_data)
        self.create_subscription(LaserScan,'/scan',self.scan,qos_profile_sensor_data)
    def stamped_phase(self,m):
        data=json.loads(m.data)
        for t,phase in data['samples']:self.phase_samples[int(t)]=phase
        # Keep the entire current lap plus 100 s for the next start.
        if self.phase_samples and self.current is None:
            newest=max(self.phase_samples);self.phase_samples={t:p for t,p in self.phase_samples.items() if newest-t<100_000_000_000}
    def scan(self,m):self.scans+=1
    def localization(self,m):
        p=m.pose.pose.position;self.estimate=(p.x,p.y,yaw(m.pose.pose.orientation),time.monotonic())
        source=m.header.stamp.sec*1_000_000_000+m.header.stamp.nanosec
        self.estimate_samples[source]=(p.x,p.y,yaw(m.pose.pose.orientation))
    def lap(self,m):
        d=json.loads(m.data);key=(d['run_id'],d['lap_id'])
        if key==self.key:return
        self.key=key;self.snapshot_freeze=False;self.current={'run_id':key[0],'lap_id':key[1],'records':[],'reached':[],'observations':[],'crossings':[],'statuses':[],'started_wall':time.monotonic(),'initial_phase':self.phase,'performance_frames':[]}
        self.vision=None;self.first_counts=None;self.complete_at=None;self.complete_events=None;self.event_checks=[]
        self.camera_stamps=[];self.image_stamps=set();self.scans=0;self.faults=[];self.active_fault=None;self.restart_rejected=None
        self.signal_diagnostics=[];self.signal_decisions=[];self.stress_statistics=[]
        self.localization_quality=[];self.approach_decisions=[];self.signal_holds=[]
        print('AUDIT_LAP_START',json.dumps(d),flush=True)
    def on_status(self,m):
        self.status=m.data
        if self.current is None:return
        statuses=self.current['statuses']
        if not statuses or statuses[-1]['status']!=m.data:
            statuses.append({'wall_s':time.monotonic()-self.current['started_wall'],'status':m.data})
        if m.data.startswith('REACHED '):
            name=m.data.split()[1]
            if not self.current['reached'] or self.current['reached'][-1]!=name:
                self.current['reached'].append(name);print('AUDIT_REACHED',self.key[1],name,flush=True)
    def observation(self,m):
        if self.current is None:return
        d=json.loads(m.data);view=d.get('view')
        if d.get('lap_id')!=self.key[1] or view=='transit':return
        if not self.current['observations'] or self.current['observations'][-1]!=view:self.current['observations'].append(view)
    def perception(self,m):
        d=json.loads(m.data)
        if self.current is None or (d.get('run_id'),d.get('lap_id'))!=self.key:return
        self.vision=d
        self.current['performance_frames'].append({'wall':time.monotonic(), 'stamp':d['stamp'], 'processing_ms':d.get('inference_and_ocr_ms'), 'image_age_sim_s':self.pose[3]-d['stamp'] if self.pose else None,
            'view':d.get('view'),'feedback':d.get('observation_feedback'),
            'people_boxes':[b for b in d.get('detections',[]) if b.get('category') in ('resident','visitor')]})
        if self.first_counts is None:self.first_counts=copy.deepcopy(d['counts'])
        for e in d['events'][len(self.event_checks):]:
            candidates=[b for b in d['detections'] if b['name']==e['content'] and b['bbox']==e.get('bbox')]
            exact=any(abs(float(b.get('ocr_confidence') if b.get('ocr_confidence') is not None else b['confidence'])-e['confidence'])<.0002 for b in candidates)
            self.event_checks.append({'sequence':e['sequence'],'frame_id':e['frame_id'],'stamp':e['stamp'],'same_frame':e['frame_id']==d['frame_id'] and abs(e['stamp']-d['stamp'])<1e-6,'matching_box_name_confidence':exact,'image':e['image']})
        if d['status'].startswith('COMPLETE') and self.complete_at is None:
            self.complete_at=time.monotonic();self.complete_events=copy.deepcopy(d['events'])
    def truth(self,m):
        if self.snapshot_freeze:return
        p=m.pose.pose.position;q=m.pose.pose.orientation;a=yaw(q);t=stamp(m.header);now=time.monotonic()
        speed=math.hypot(m.twist.twist.linear.x,m.twist.twist.linear.y)
        self.pose=(p.x,p.y,a,t,speed)
        if self.current is None:return
        points=footprint(p.x,p.y,a);outside=any(not in_road(x,y,.004) for x,y in points)
        front=[min(x for x,y in points)-.30,min(y for x,y in points)+.51]
        est=self.estimate;fresh=est is not None and now-est[3]<.7
        error=[math.hypot(p.x-est[0],p.y-est[1]),abs(wrap(a-est[2]))] if fresh else None
        r={'t':t,'wall_s':now-self.current['started_wall'],'pose':[p.x,p.y,a],'speed':speed,'command':self.command,'status':self.status,'phase':self.phase,'front':front,'outside':outside,'fresh_pose_error':error}
        records=self.current['records']
        if records:
            previous=records[-1]
            for i in range(2):
                near=abs(p.y-1.7)<.30 if i==0 else abs(p.x+.1)<.30
                if near and previous['front'][i]>=0 and front[i]<0:
                    self.current['crossings'].append({'gate':i+1,'time':t,'phase':self.phase,'legal':self.phase=='green'})
        records.append(r)
        if self.active_fault:
            self.active_fault['samples'].append({'elapsed':now-self.active_fault['start_wall'],'pose':[p.x,p.y],'speed':speed,'command':self.command,'status':self.status})
    def fault_tick(self):
        if self.args.fault_lap!=self.key[1]:return
        now=time.monotonic()
        if self.active_fault:
            if now-self.active_fault['start_wall']>=2.5:
                f=self.active_fault;os.kill(f['pid'],signal.SIGCONT);self.paused.pop(f['pid'],None)
                late=[s for s in f['samples'] if s['elapsed']>=1.5]
                f['settled_command_zero']=bool(late) and all(abs(s['command'][0])<.001 and abs(s['command'][1])<.001 for s in late)
                f['settled_speed_max']=max((s['speed'] for s in late),default=999.)
                f['passed']=f['settled_command_zero'] and f['settled_speed_max']<.015
                f['first_zero_command_wall_s']=next((s['elapsed'] for s in f['samples'] if max(map(abs,s['command']))<.001),None)
                f['first_settled_wall_s']=next((s['elapsed'] for s in f['samples'] if s['speed']<.015 and max(map(abs,s['command']))<.001),None)
                f['duration_wall_s']=now-f['start_wall'];f.pop('start_wall');self.faults.append(f);self.active_fault=None
                print('FAULT_RESUMED',f['node'],f['passed'],flush=True)
            return
        done={f['node'] for f in self.faults};x,y,a,t,speed=self.pose or (0,0,0,0,0)
        target=None
        if 'perception' not in done and ((self.args.moving_faults and self.status.startswith('GO top_west') and speed>.07) or (not self.args.moving_faults and self.status.startswith('STOP top_stop'))):target='perception'
        elif 'localization' not in done and self.status.startswith('GO lower_south') and -1.10<y<-.80 and speed>.07:target='localization'
        elif 'patrol' not in done and self.status.startswith('GO right_north') and .1<y<.5 and speed>.07:target='patrol'
        if target:
            pid=process_for(target);self.active_fault={'node':target,'pid':pid,'start_wall':now,'start_sim':t,'initial_speed_m_s':speed,'samples':[]};self.paused[pid]=target
            os.kill(pid,signal.SIGSTOP);print('FAULT_PAUSED',target,pid,flush=True)
    def snapshot(self):
        c=self.current;records=c['records']
        # Freeze the physical record boundary and let delayed source-stamped
        # evidence arrive; do not extend records or change the age tolerance.
        self.snapshot_freeze=True;latest=round(records[-1]['t']*1e9);deadline=time.monotonic()+.5
        while time.monotonic()<deadline and (not self.phase_samples or max(self.phase_samples)<latest):
            rclpy.spin_once(self,timeout_sec=.03)
        pose_times=sorted(self.estimate_samples);aligned=0
        for record in records:
            ns=round(record['t']*1e9);estimate=pose_at_source(self.estimate_samples,pose_times,ns)
            record['receipt_pose_error']=record['fresh_pose_error']
            record['fresh_pose_error']=None if estimate is None else [math.hypot(record['pose'][0]-estimate[0],record['pose'][1]-estimate[1]),abs(wrap(record['pose'][2]-estimate[2]))]
            aligned+=estimate is not None
        phase_ages=align_signal_evidence(records,self.phase_samples)
        # Recompute physical line crossings from source-time-aligned lens evidence.
        c['crossings']=[]
        for previous,r in zip(records,records[1:]):
            x,y,_=r['pose']
            for i,near in enumerate((abs(y-1.7)<.30,abs(x+.1)<.30)):
                if near and previous['front'][i]>=0 and r['front'][i]<0:
                    c['crossings'].append({'gate':i+1,'time':r['t'],'phase':r['phase'],'legal':r['phase']=='green'})
        v=self.vision or {};events=v.get('events',[])
        layout=json.loads((ROOT/'src/community_demo/worlds/layout.json').read_text());expected={p['asset'] for p in layout['people']}
        distance=sum(math.hypot(b['pose'][0]-a['pose'][0],b['pose'][1]-a['pose'][1]) for a,b in zip(records,records[1:]))
        errors=[r['fresh_pose_error'] for r in records if r['fresh_pose_error']]
        waits=[]
        for gate,name in [(0,'top_stop'),(1,'lower_stop')]:
            subset=[r for r in records if r['status'].startswith('STOP '+name) and r['phase']=='red']
            waits.append({'gate':gate+1,'red_wait_samples':len(subset),'min_front_clearance_m':min((r['front'][gate] for r in subset),default=None),'max_speed_m_s':max((r['speed'] for r in subset),default=None)})
        for e in self.event_checks:
            e['annotated_topic_frame_seen']=round(e['stamp'],6) in self.image_stamps
            path=ROOT/'log/perception'/e['image'];e['image_exists']=path.is_file()
            if path.is_file():e['sha256']=hashlib.sha256(path.read_bytes()).hexdigest()
        views=json.loads((ROOT/'log/inspection/views.json').read_text())
        outcomes=json.loads((ROOT/'log/inspection/outcomes.json').read_text())
        whole_body=[];previous_edges=[None,None];red_straddles=0
        for r in records:
            x,y,a=r['pose'];body=footprint(x,y,a)
            edges=[(min(px for px,py in body)-.3,max(px for px,py in body)-.3),(min(py for px,py in body)+.51,max(py for px,py in body)+.51)]
            for gate,near in enumerate([abs(y-1.7)<.3,abs(x+.1)<.3]):
                if not near:previous_edges[gate]=None;continue
                if edges[gate][0]<0<edges[gate][1] and r['phase']=='red':red_straddles+=1
                if previous_edges[gate]:
                    for edge,label in enumerate(['front_entered','rear_cleared']):
                        if previous_edges[gate][edge]>=0 and edges[gate][edge]<0:whole_body.append({'gate':gate+1,'edge':label,'time':r['t'],'phase':r['phase']})
                previous_edges[gate]=edges[gate]
        tail=[r for r in records if self.complete_at and c['started_wall']+r['wall_s']>=self.complete_at+1]
        checks={'pose_source_time_coverage':aligned/len(records)>=(.90 if self.stress_statistics else .98),'signal_source_time_alignment':bool(phase_ages) and all(x is not None and x<=.075 for x in phase_ages) and all(r['phase']!='unknown' for r in records),'completed':v.get('status','').startswith('COMPLETE'),'route_order':c['reached']==EXPECTED_ROUTE,'actual_distance':distance>12 and distance<15,'returned_to_start':bool(records) and math.hypot(records[-1]['pose'][0]-START[0],records[-1]['pose'][1]-START[1])<.10 and in_start_bay(body_corners(*records[-1]['pose'])),'road_boundary':not any(r['outside'] for r in records),'two_green_crossings':len(c['crossings'])==2 and all(x['legal'] for x in c['crossings']),'red_wait_body_before_line':all(x['min_front_clearance_m'] is None or x['min_front_clearance_m']>=0 for x in waits),'eight_observations':c['observations']==EXPECTED_VIEWS,'observation_log_reset':len(views)==8 and all(x.get('lap_id')==self.key[1] and x.get('run_id')==self.key[0] for x in views),'recognition_reset':self.first_counts=={'residents':0,'visitors':0,'total':0},'people':set(v.get('people',{}))==expected,'plates':set(v.get('plates',{}))==EXPECTED_PLATES,'event_sequence':[e['sequence'] for e in events]==list(range(1,len(events)+1)),'event_frame_correspondence':bool(self.event_checks) and all(e['same_frame'] and e['matching_box_name_confidence'] and e['annotated_topic_frame_seen'] and e['image_exists'] for e in self.event_checks),'completion_frozen':events==self.complete_events,'stationary_after_completion':bool(tail) and max(r['speed'] for r in tail)<.015 and all(abs(r['command'][0])<.001 and abs(r['command'][1])<.001 for r in tail),'camera_and_scan_live':len(self.camera_stamps)>100 and self.scans>100,'restart_while_running_rejected':self.restart_rejected is True}
        if self.args.fault_lap==self.key[1]:checks['fault_stop_and_recovery']=len(self.faults)==3 and all(f['passed'] for f in self.faults)
        checks.update(front_and_rear_green=len(whole_body)==4 and all(x['phase']=='green' for x in whole_body),red_straddle_samples_zero=red_straddles==0,observation_confirmation=len(outcomes)==8 and all(x['confirmed'] for x in outcomes))
        extra={'pose_metric':'Wheel-source-time-aligned estimate; exact samples or bracketed interpolation <=150 ms, no extrapolation; receipt metric preserved separately','pose_source_time_coverage':aligned/len(records),'estimate_source_samples':[[t,*pose] for t,pose in sorted(self.estimate_samples.items()) if records[0]['t']-.2<=t/1e9<=records[-1]['t']+.2],'signal_pose_samples':[[t,p] for t,p in sorted(self.phase_samples.items()) if records[0]['t']-.1<=t/1e9<=records[-1]['t']+.1],'whole_body_crossings':whole_body,'red_straddle_samples':red_straddles,'observation_outcomes':outcomes,
               'signal_holds':self.signal_holds,'signal_diagnostics':self.signal_diagnostics,'signal_decisions':self.signal_decisions,
               'stress_statistics':self.stress_statistics,'localization_quality':self.localization_quality,'approach_decisions':self.approach_decisions}
        posts=json.loads((ROOT/'src/community_demo/maps/planning-obstacles.json').read_text())['posts']
        clearance=[]
        for r in records:
            x,y,a=r['pose'];co,si=math.cos(a),math.sin(a)
            for p in posts:
                dx=p['x']-x;dy=p['y']-y
                clearance.append(math.hypot(max(abs(co*dx+si*dy)-.11,0),max(abs(-si*dx+co*dy)-.1125,0))-math.hypot(p['width']/2,p['depth']/2))
        checks['physical_post_clearance']=bool(clearance) and min(clearance)>=-.002
        extra['minimum_post_clearance_m']=min(clearance) if clearance else None
        return {**extra,'performance_frames':c['performance_frames'],'passed':all(checks.values()),'run_id':self.key[0],'lap_id':self.key[1],'mode':'fault injection' if self.args.fault_lap==self.key[1] else 'normal autonomous lap','checks':checks,'sim_elapsed_s':records[-1]['t']-records[0]['t'] if records else 0,'physical_distance_m':distance,'waypoints':c['reached'],'observations':c['observations'],'counts':v.get('counts'),'plates':list(v.get('plates',{})),'initial_phase':records[0]['phase'] if records else 'unknown','initial_phase_at_receipt':c['initial_phase'],'initial_phase_source_ns':round(records[0]['t']*1e9) if records else None,'crossings':c['crossings'],'red_waits':waits,'max_fresh_position_error_m':max((x[0] for x in errors),default=None),'max_fresh_yaw_error_deg':math.degrees(max((x[1] for x in errors),default=0)),'road_violation_samples':sum(r['outside'] for r in records),'samples':len(records),'camera_frames':len(self.camera_stamps),'scan_frames':self.scans,'event_checks':self.event_checks,'faults':self.faults,'status_changes':c['statuses'],'records':records,'perception':v}
    def cleanup(self):
        for pid in list(self.paused):
            try:os.kill(pid,signal.SIGCONT)
            except ProcessLookupError:pass
        self.paused.clear()

def spin_until(n,predicate,timeout):
    end=time.monotonic()+timeout
    while rclpy.ok() and time.monotonic()<end:
        rclpy.spin_once(n,timeout_sec=.05)
        if predicate():return True
    return False

def request_restart(n):
    if not n.restart_client.wait_for_service(timeout_sec=5):raise RuntimeError('Restart service missing')
    future=n.restart_client.call_async(Trigger.Request())
    if not spin_until(n,future.done,8):raise RuntimeError('Restart response timeout')
    return future.result()

def wait_restart_phase(n,phase,delay):
    # This controls the test's start time, never supplies signal state to patrol.
    if not phase:return
    if not spin_until(n,lambda:n.phase!=phase,35):raise RuntimeError('Signal phase did not change')
    if not spin_until(n,lambda:n.phase==phase,35):raise RuntimeError('Requested restart phase missing')
    start=time.monotonic()
    if delay and not spin_until(n,lambda:time.monotonic()-start>=delay,delay+3):raise RuntimeError('Phase delay timed out')
    print('RESTART_PHASE',phase,'delay',delay,flush=True)

def main():
    p=argparse.ArgumentParser();p.add_argument('--laps',type=int,default=3);p.add_argument('--fault-lap',type=int,default=3);p.add_argument('--unpause',action='store_true');p.add_argument('--moving-faults',action='store_true');p.add_argument('--restart',action='store_true');p.add_argument('--restart-phases',default='');p.add_argument('--green-delay',type=float,default=10.);p.add_argument('--green-delays',default='');p.add_argument('--output',default='validation/patrol-cycle-review');p.add_argument('--timeout',type=float,default=300);a=p.parse_args()
    out=ROOT/a.output;out.mkdir(parents=True,exist_ok=True);rclpy.init(signal_handler_options=SignalHandlerOptions.NO);n=Audit(a);reports=[]
    try:
        if not spin_until(n,lambda:n.key is not None,50):raise RuntimeError('No explicit patrol lap lifecycle')
        if a.unpause:
            subprocess.run(['gz','service','-s','/world/community/control','--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','5000','--req','pause: false'],check=True,capture_output=True)
        if a.restart:
            previous=n.key;response=request_restart(n)
            if not response.success:raise RuntimeError(response.message)
            if not spin_until(n,lambda:n.key!=previous,5):raise RuntimeError('New lap lifecycle not received')
        for number in range(a.laps):
            begin=time.monotonic();checked_restart=False
            while time.monotonic()-begin<a.timeout:
                rclpy.spin_once(n,timeout_sec=.05)
                if n.key:n.fault_tick()
                if not checked_restart and n.current and len(n.current['reached'])>=3:
                    response=request_restart(n);n.restart_rejected=not response.success;checked_restart=True
                    if response.success:raise RuntimeError('Running lap restart unexpectedly accepted')
                if n.status.startswith('FAULT'):raise RuntimeError(n.status)
                if n.complete_at and time.monotonic()-n.complete_at>4:break
            if not n.complete_at:raise RuntimeError('Lap timed out: '+n.status)
            report=n.snapshot();filename=f'lap-{n.key[1]:03d}.json';(out/filename).write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
            image_report_name=f'lap-{n.key[1]:03d}-images.json'
            env=os.environ.copy();env['PYTHONPATH']=str(ROOT/'.deps')+os.pathsep+env.get('PYTHONPATH','')
            verification=subprocess.run(['/home/ubuntu/.local/share/aic-vision-venv/bin/python',str(ROOT/'tools/verify_event_images.py'),'--report',str(out/filename),'--output',str(out/image_report_name)],env=env,capture_output=True,text=True,timeout=90)
            (ROOT/'log'/f'cycle-image-check-{n.key[1]}.log').write_text(verification.stdout+verification.stderr)
            images=json.loads((out/image_report_name).read_text())
            report['raw_image_topic_event_frames_seen']=sum(e['annotated_topic_frame_seen'] for e in report['event_checks'])
            report['checks']['rendered_image_frame_ids']=verification.returncode==0 and images['passed']
            report['checks']['event_frame_correspondence']=report['checks']['rendered_image_frame_ids'] and all(e['same_frame'] and e['matching_box_name_confidence'] and e['image_exists'] for e in report['event_checks'])
            report['image_correspondence_method']='Rendered JPEG frame number independently read with OCR; best-effort raw topic reception remains a diagnostic, not a guarantee of delivery'
            report['image_report']=image_report_name;report['passed']=all(report['checks'].values())
            terminal_events=[]
            for line in (ROOT/'log/perception-terminal.log').read_text().splitlines():
                start=line.find('{"run_id":')
                if start<0:continue
                try:e=json.loads(line[start:])
                except ValueError:continue
                if (e.get('run_id'),e.get('lap_id'))==n.key:terminal_events.append(e)
            report['checks']['terminal_events_exact']=terminal_events==report['perception']['events']
            report['passed']=all(report['checks'].values())
            (out/filename).write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
            # Photos stay in ignored log/, while text evidence is kept per lap.
            archive=ROOT/'log/patrol-cycle-review'/n.key[0]/f'lap-{n.key[1]:03d}';archive.mkdir(parents=True,exist_ok=True)
            import shutil
            for e in report['perception']['events']:shutil.copy2(ROOT/'log/perception'/e['image'],archive/e['image'])
            compact={k:v for k,v in report.items() if k not in ['records','perception','event_checks','status_changes','faults','performance_frames','signal_pose_samples','estimate_source_samples']};compact['report']=filename;reports.append(compact)
            print('LAP_RESULT',json.dumps(compact,ensure_ascii=False),flush=True)
            if not report['passed']:break
            if number<a.laps-1:
                phases=a.restart_phases.split(',') if a.restart_phases else []
                phase=phases[number%len(phases)] if phases else ''
                delays=[float(x) for x in a.green_delays.split(',')] if a.green_delays else []
                delay=delays[number%len(delays)] if delays else a.green_delay
                wait_restart_phase(n,phase,delay if phase=='green' else 0.)
                previous=n.key;response=request_restart(n)
                if not response.success:raise RuntimeError(response.message)
                if not spin_until(n,lambda:n.key!=previous,5):raise RuntimeError('New lap lifecycle not received')
        subscribers={topic:[x.node_name for x in n.get_subscriptions_info_by_topic(topic)] for topic in ['/model/patrol_bot/ground_truth','/simulation/traffic_light_state']}
        summary={'passed':len(reports)==a.laps and all(r['passed'] for r in reports),'requested_laps':a.laps,'laps':reports,'evaluation_only_subscribers':subscribers,'limits':['One fixed supplied-gallery scene','Road footprint evaluated at sampled ground-truth poses','Fault lap is labelled and excluded from normal autonomous-lap count','No altered training weights or thresholds'],'local_photo_root':str(ROOT/'log/patrol-cycle-review')}
        (out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n');print('CYCLE_AUDIT_DONE',summary['passed'],flush=True)
        return 0 if summary['passed'] else 1
    except (Exception,KeyboardInterrupt) as exc:
        partial=n.snapshot() if n.current and n.current['records'] else {}
        partial.update(passed=False,error=str(exc))
        (out/'failure.json').write_text(json.dumps(partial,ensure_ascii=False,indent=2)+'\n')
        raise
    finally:n.cleanup();n.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':raise SystemExit(main())
