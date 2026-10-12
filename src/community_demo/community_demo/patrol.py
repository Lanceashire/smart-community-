"""Nav2-planned single-lap patrol, camera stop gates and explicit lap lifecycle."""

import math
import json
import uuid
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import cv2
import numpy as np

import rclpy
from geometry_msgs.msg import TwistStamped,PoseStamped
from nav2_msgs.action import ComputePathToPose
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from nav_msgs.msg import Odometry,OccupancyGrid,Path as NavPath
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data,ReliabilityPolicy
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import String, Float64
from std_srvs.srv import Trigger

from community_demo.bounded_pid import BoundedPID
from community_demo.road_rules import safe_pose,predict,footprint,clear_of_posts,corridor_recovery
from ament_index_python.packages import get_package_share_directory
from community_demo.signal_gate import FreshGreenGate, VisualPhaseClock
from community_demo.motion_profile import path_velocity,velocity_envelope,coupled_command
from community_demo.sensor_validation import valid_rgb_image,valid_odometry,stamp_ns,valid_laser_scan
from community_demo.inspection_policy import person_evidence_ready,signal_hold_reason,inspection_complete,recognition_complete
from community_demo.continuous_route import batch_end,passage,plan_batch,rebase_batch
from community_demo.route_path import GridClearance,curve_target,pivot_clear,rolling_terminal_allowed
from community_demo.task_geometry import TASK,START,in_start_bay
from community_demo.road_rules import body_corners


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


# Task geometry and semantic stops are data; every path uses one solver.
WAYPOINTS=TASK['waypoints']
ROUTE=[(w['name'],*w['position'],w['stop_for_light']) for w in WAYPOINTS]
OBSERVATIONS={w['name']:w['views'] for w in WAYPOINTS if w['views']}
OBSERVATION_HEADING={w['name']:w['heading'] for w in WAYPOINTS if w['views']}



class Patrol(Node):
    def __init__(self):
        super().__init__('community_patrol')
        self.declare_parameter('require_vision', True)
        self.declare_parameter('pose_topic','/localization/pose')
        # The supplied sketch has five standees in each district and three cars.
        self.declare_parameter('expected_people',10)
        self.declare_parameter('expected_plates',3)
        self.expected_people=self.get_parameter('expected_people').value
        self.expected_plates=self.get_parameter('expected_plates').value
        self.recognition_counts={};self.recognition_plates=[]
        self.require_vision = self.get_parameter('require_vision').value
        latest=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pan_pub=self.create_publisher(Float64,'/model/patrol_bot/camera_pan',10)
        self.observation_pub=self.create_publisher(String,'/inspection/observation',10)
        self.create_subscription(Image,'/camera/image',self.on_camera,latest)
        self.camera_frame=None
        self.observation_start=None
        self.observation_align=False
        self.observation_index=0
        self.observation_attempt=0
        self.observation_sample=0
        self.sample_start=None
        self.observation_feedback={}
        self.feedback_update=None
        self.perception_update=None;self.perception_stamp=None
        self.observation_outcomes=[]
        self.create_subscription(String,'/perception/results',self.on_perception,latest)
        self.inspection_pub=self.create_publisher(String,'/inspection/outcome',10)
        self.static_posts=json.loads((Path(get_package_share_directory('community_demo'))/'maps/planning-obstacles.json').read_text())['posts']
        self.post_replan_time=-1e9
        self.road_recovery_started=None
        self.road_recovery_origin=None
        self.localization_quality={}
        self.quality_update=None
        self.blocked_since=None
        self.clear_scans=0
        self.recovery_count=0
        self.create_subscription(String,'/localization/quality',self.on_quality,latest)
        self.saved_views=[]
        inspection=Path('log/inspection');inspection.mkdir(parents=True,exist_ok=True)
        (inspection/'views.json').write_text('[]\n')
        (inspection/'outcomes.json').write_text('[]\n')
        self.path_preparer=ThreadPoolExecutor(max_workers=1,thread_name_prefix="path_prepare")
        self.planner=ActionClient(self,ComputePathToPose,'/compute_path_to_pose')
        self.path_request_token=0;self.path_request_deadline=None;self.path_goal_handle=None
        self.path_retry_index=None;self.path_attempts=0
        self.path_retry_deadline=None;self.path_retry_grid_ns=None
        self.path=None;self.path_pending=False;self.path_index=-1;self.path_pub=self.create_publisher(NavPath,'/patrol/path',10)
        self.raw_path_pub=self.create_publisher(NavPath,'/patrol/raw_path',10)
        self.costmap=None;self.costmap_stamp_ns=None
        self.create_subscription(OccupancyGrid,'/global_costmap/costmap',self.on_costmap,
                                 QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.path_refined=False;self.path_segment=1;self.path_align=False;self.path_settle_until=0.
        self.path_curved=False;self.curve_progress=0;self.path_curvature=0.
        self.path_goal_yaw=None
        self.path_end_index=0
        self.inspection_done_index=None
        self.last_motion_pose=None
        self.command_pub = self.create_publisher(TwistStamped, '/cmd_vel_nav', 10)
        self.status_pub = self.create_publisher(String, '/patrol/status', 10)
        self.signal_decisions = self.create_publisher(String, '/patrol/signal_decision', 10)
        self.signal_hold_pub = self.create_publisher(String, '/patrol/signal_hold', 10)
        self.approach_decisions = self.create_publisher(String, '/patrol/approach_decision', 10)
        self.control_trace_pub=self.create_publisher(String,'/patrol/control_trace',10)
        self.lap_pub = self.create_publisher(String, '/patrol/lap', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.run_id = uuid.uuid4().hex[:12]
        self.lap_id = 1
        # Control consumes camera/wheel localization. Simulator truth is evaluation only.
        self.create_subscription(Odometry, self.get_parameter('pose_topic').value, self.on_odom, latest)
        self.create_service(Trigger, '/patrol/restart', self.restart)
        self.create_subscription(LaserScan, '/scan', self.on_scan, latest)
        self.create_subscription(String, '/perception/traffic_light_stamped', self.on_stamped_light, latest)
        self.pose = None
        self.pose_update = None
        self.pose_source_time=None
        self.pose_source_ns=None
        self.tilt = 0.0
        self.distance_travelled = 0.0
        self.motion_probe = None
        self.fault = None
        self.scan = None
        self.scan_update = None
        self.scan_source_ns = None
        self.light_state = 'unknown'
        self.light_update = None
        self.light_source_ns = None
        self.index = 0
        self.green_streak = 0
        self.waiting_for_green = False
        self.phase_clock = VisualPhaseClock()
        self.signal_gate = FreshGreenGate(self.phase_clock)
        self.signal_context = None
        self.crossing_gate = None
        self.completed = False
        self.last_log = ''
        self.speed_pid = BoundedPID(kp=.70, lower=0.0, upper=.26)
        self.heading_pid = BoundedPID(kp=1.8, lower=-.65, upper=.65)
        self.create_timer(.1, self.tick)
        self.get_logger().info('Single-lap route ready; uses camera / wheel localization with separate laser SLAM mapping and camera-confirmed green.')
        self.publish_lap('started')

    def publish_lap(self, phase):
        self.lap_pub.publish(String(data=json.dumps({'run_id': self.run_id, 'lap_id': self.lap_id, 'phase': phase})))

    def cancel_path_request(self):
        self.path_request_token += 1
        handle=self.path_goal_handle
        self.path_goal_handle=None;self.path_request_deadline=None
        self.path_pending=False
        if handle is not None:
            try:handle.cancel_goal_async()
            except Exception as exc:self.get_logger().warning('Nav2 cancel failed: '+str(exc))

    def fail_path_request(self,reason):
        self.cancel_path_request()
        self.path=None;self.path_index=-1
        if self.path_attempts >= 3:
            self.fault='FAULT Nav2 planning failed after three attempts: '+reason
        else:
            # Do not spend every retry on one pre-clearance costmap. The
            # full grid is published more slowly than scans or patrol ticks.
            self.path_retry_grid_ns=getattr(self,'costmap_stamp_ns',None)
            self.path_retry_deadline=self.get_clock().now().nanoseconds/1e9+5.
            self.get_logger().warning('Nav2 bounded retry: '+reason)

    def path_retry_ready(self,now):
        deadline=getattr(self,'path_retry_deadline',None)
        if deadline is None:return True
        if now.nanoseconds/1e9>=deadline:
            self.fault='FAULT no updated live grid within bounded planning retry'
            return False
        before=getattr(self,'path_retry_grid_ns',None)
        stamp=getattr(self,'costmap_stamp_ns',None)
        if stamp is None or (before is not None and stamp<=before):return False
        self.path_retry_deadline=None;self.path_retry_grid_ns=None
        return True

    def check_path_deadline(self,now):
        if (self.path_pending and self.path_request_deadline is not None and
                now.nanoseconds/1e9 >= self.path_request_deadline):
            self.fail_path_request('five-second request timeout')

    def request_path(self,tx,ty):
        if self.path_retry_index != self.index:
            self.path_retry_index=self.index;self.path_attempts=0
            self.path_retry_deadline=None;self.path_retry_grid_ns=None
        if not self.path_retry_ready(self.get_clock().now()):return
        self.cancel_path_request()
        self.path_attempts += 1
        self.path_pending=True;self.path_index=self.index
        self.path_request_deadline=self.get_clock().now().nanoseconds/1e9+5.
        token=self.path_request_token;route_index=self.index
        end_index=batch_end(WAYPOINTS,route_index)
        tx,ty=WAYPOINTS[end_index]['position']
        if not self.planner.server_is_ready():return
        goal=ComputePathToPose.Goal();goal.goal=PoseStamped()
        goal.goal.header.frame_id='field';goal.goal.header.stamp=self.get_clock().now().to_msg()
        goal.goal.pose.position.x=float(tx);goal.goal.pose.position.y=float(ty)
        heading=WAYPOINTS[end_index]['heading']
        goal.goal.pose.orientation.z=math.sin(heading/2);goal.goal.pose.orientation.w=math.cos(heading/2);goal.planner_id='GridBased'
        def current():
            return token == self.path_request_token and route_index == self.index and not self.completed
        def accepted(f):
            try:
                handle=f.result()
                if not current():
                    if handle is not None and handle.accepted:handle.cancel_goal_async()
                    return
                if handle is None or not handle.accepted:
                    self.fail_path_request('planning goal rejected');return
                self.path_goal_handle=handle
                def result(g):
                    if not current():return
                    try:
                        response=g.result();path=response.result.path
                        if response.status != GoalStatus.STATUS_SUCCEEDED or not path.poses:
                            self.fail_path_request('no successful drivable path');return
                        if not all(math.isfinite(p.pose.position.x) and math.isfinite(p.pose.position.y)
                                   for p in path.poses):
                            self.fail_path_request('nonfinite path');return
                        self.install_path(path,route_index,end_index)
                    except Exception as exc:self.fail_path_request('result exception: '+str(exc))
                handle.get_result_async().add_done_callback(result)
            except Exception as exc:
                if current():self.fail_path_request('goal exception: '+str(exc))
        try:self.planner.send_goal_async(goal).add_done_callback(accepted)
        except Exception as exc:self.fail_path_request('send exception: '+str(exc))

    def install_path(self,path,route_index,end_index,prepared=None):
        """Certify from the current measured pose and current grid on every handover."""
        self.path_pending=False;self.path_request_deadline=None;self.path_goal_handle=None
        self.path_refined=False
        self.path_curved=False;self.curve_progress=0;self.path_goal_yaw=None
        if hasattr(self,'raw_path_pub'):self.raw_path_pub.publish(path)
        now_ns=self.get_clock().now().nanoseconds
        grid=getattr(self,'costmap',None);grid_stamp=getattr(self,'costmap_stamp_ns',None)
        points=[(p.pose.position.x,p.pose.position.y) for p in path.poses]
        route_name=ROUTE[end_index][0]
        fresh=(grid is not None and grid_stamp is not None and 0<=now_ns-grid_stamp<=2_000_000_000)
        if not fresh:
            self.fail_path_request('every trajectory requires a fresh live grid');return
        goal_yaw=WAYPOINTS[end_index]['heading']
        curve,planning=(None,{})
        if prepared is not None and prepared[0] is not None:
            curve,planning=rebase_batch(*prepared,self.pose,grid,self.static_posts,TASK['trajectory'].get('minimum_radius',.08))
        if curve is None:
            fallback=planning if prepared is not None else {'rejected':'no completed prepared curve'}
            curve,planning=plan_batch(points,self.pose,WAYPOINTS,route_index,end_index,grid,self.static_posts,TASK['trajectory'])
            planning['handover_fallback']=fallback
        if curve is None:
            out=Path('log/planning-rejections')/self.run_id;out.mkdir(parents=True,exist_ok=True)
            (out/f'lap-{self.lap_id:03d}-{route_name}-{now_ns}.json').write_text(json.dumps({
                'pose':self.pose,'goal_heading':goal_yaw,'raw_points':points,
                'planning':planning,'stamp_ns':now_ns,'grid_source_ns':grid_stamp,
                'grid':{'width':grid.width,'height':grid.height,'resolution':grid.resolution,
                    'origin':grid.origin,'data':list(grid.data)}})+'\n')
            self.fail_path_request('no whole-body-safe continuous trajectory');return
        self.path_velocity_envelope=velocity_envelope(curve,TASK['motion'])
        if self.path_velocity_envelope is None:
            self.fail_path_request('invalid arc-distance retiming');return
        refined=[p[:2] for p in curve]
        clean=NavPath();clean.header=path.header
        for x,y,z in curve:
            p=PoseStamped();p.header=path.header;p.pose.position.x=float(x);p.pose.position.y=float(y)
            p.pose.orientation.z=math.sin(z/2);p.pose.orientation.w=math.cos(z/2);clean.poses.append(p)
        path=clean;self.path_refined=True;self.path_curved=True;self.path_goal_yaw=goal_yaw
        self.path_end_index=end_index
        out=Path('log/paths')/self.run_id;out.mkdir(parents=True,exist_ok=True)
        evidence={'run_id':self.run_id,'lap_id':self.lap_id,'stamp_ns':now_ns,
            'pose':self.pose,'pose_source_ns':self.pose_source_ns,'grid_source_ns':grid_stamp,
            'grid_age_s':(now_ns-grid_stamp)/1e9,'raw_points':points,'refined_points':refined,
            'strategy':'unified_continuous_curvature','planning':planning,'curve_poses':curve,
            'velocity_envelope':self.path_velocity_envelope,
            'grid':{'width':grid.width,'height':grid.height,'resolution':grid.resolution,
                'origin':grid.origin,'data':list(grid.data)}}
        (out/f'lap-{self.lap_id:03d}-{route_name}-{now_ns}.json').write_text(json.dumps(evidence)+'\n')
        self.get_logger().info(f'PATH_REFINED {route_name}: {len(points)} -> {len(curve)}; shared continuous-curvature solver')
        self.path_attempts=0
        self.path_align=self.path_refined and not self.path_curved;self.path_settle_until=now_ns/1e9+.35
        self.path=path;self.path_pub.publish(path)

    def prefetch_next(self,inspection_index,now):
        """Ask Nav2 while sensing; no cached path is ever sent to the chassis."""
        self.promote_prepared_path()
        if inspection_index+1>=len(WAYPOINTS) or not self.planner.server_is_ready():return
        key=(self.lap_id,inspection_index+1)
        if getattr(self,'prefetch_key',None)==key and now.nanoseconds-getattr(self,'prefetch_stamp_ns',0)<4_000_000_000:return
        self.prefetch_key=key;self.prefetch_stamp_ns=now.nanoseconds
        token=(key,now.nanoseconds)
        end=batch_end(WAYPOINTS,inspection_index+1);point=WAYPOINTS[end]
        goal=ComputePathToPose.Goal();goal.goal=PoseStamped();goal.goal.header.frame_id='field';goal.goal.header.stamp=now.to_msg()
        goal.goal.pose.position.x=float(point['position'][0]);goal.goal.pose.position.y=float(point['position'][1])
        goal.goal.pose.orientation.z=math.sin(point['heading']/2);goal.goal.pose.orientation.w=math.cos(point['heading']/2);goal.planner_id='GridBased'
        # Explicit future start only guides Nav2. install_path re-solves from
        # actual measured pose; this predicted start is never navigation truth.
        start=WAYPOINTS[inspection_index];goal.use_start=True;goal.start=PoseStamped();goal.start.header=goal.goal.header
        goal.start.pose.position.x=float(start['position'][0]);goal.start.pose.position.y=float(start['position'][1])
        goal.start.pose.orientation.z=math.sin(start['heading']/2);goal.start.pose.orientation.w=math.cos(start['heading']/2)
        def current():
            return token==(getattr(self,'prefetch_key',None),getattr(self,'prefetch_stamp_ns',None)) and self.lap_id==key[0] and self.index<=inspection_index+1 and not self.completed
        def accepted(f):
            try:
                handle=f.result()
                if not handle or not handle.accepted:return
                if not current():handle.cancel_goal_async();return
                def ready(future):
                    if not current():return
                    try:
                        response=future.result();path=response.result.path
                        if response.status==GoalStatus.STATUS_SUCCEEDED and path.poses and all(math.isfinite(p.pose.position.x)and math.isfinite(p.pose.position.y)for p in path.poses):
                            stamp=self.get_clock().now().nanoseconds
                            if self.costmap is None or self.costmap_stamp_ns is None or not 0<=stamp-self.costmap_stamp_ns<=2_000_000_000:return
                            points=[(p.pose.position.x,p.pose.position.y)for p in path.poses]
                            future=self.path_preparer.submit(plan_batch,points,(*start['position'],start['heading']),WAYPOINTS,inspection_index+1,end,self.costmap,self.static_posts,TASK['trajectory'])
                            self.prefetch_candidate=(key,end,path,stamp,future)
                    except Exception as exc:self.get_logger().warning('Optional path prefetch failed: '+str(exc))
                handle.get_result_async().add_done_callback(ready)
            except Exception as exc:self.get_logger().warning('Optional path prefetch failed: '+str(exc))
        try:self.planner.send_goal_async(goal).add_done_callback(accepted)
        except Exception as exc:self.get_logger().warning('Optional path prefetch failed: '+str(exc))

    def promote_prepared_path(self):
        candidate=getattr(self,'prefetch_candidate',None)
        if candidate is not None and candidate[-1].done():
            self.prefetch_candidate=None
            try:
                if candidate[-1].result()[0] is not None:self.prefetched_path=candidate
            except Exception as exc:self.get_logger().warning('Optional curve preparation failed: '+str(exc))

    def take_prefetched_path(self,now):
        if hasattr(self,'promote_prepared_path'):self.promote_prepared_path()
        cached=getattr(self,'prefetched_path',None);self.prefetched_path=None
        if cached is None:return False
        key,end,path,stamp,future=cached
        if key!=(self.lap_id,self.index) or not 0<=now.nanoseconds-stamp<=5_000_000_000 or self.front_blocked():return False
        self.path_index=self.index;self.path_attempts=0
        try:prepared=future.result() if future.done() else None
        except Exception:prepared=None
        self.install_path(path,self.index,end,prepared)
        return self.path_index==self.index and self.path is not None

    def on_costmap(self,msg):
        source=stamp_ns(msg.header.stamp)
        q=msg.info.origin.orientation
        if (source is None or msg.header.frame_id!='field' or msg.info.resolution<=0 or
                not math.isfinite(msg.info.resolution) or msg.info.width*msg.info.height!=len(msg.data) or
                not all(math.isfinite(v) for v in (msg.info.origin.position.x,msg.info.origin.position.y)) or
                not all(math.isfinite(v) for v in (q.x,q.y,q.z,q.w)) or
                abs(q.x)>1e-6 or abs(q.y)>1e-6 or abs(q.z)>1e-6 or abs(q.w-1.)>1e-6):return
        age=self.get_clock().now().nanoseconds-source
        if not 0<=age<=2_000_000_000:return
        if self.costmap_stamp_ns is not None and source<=self.costmap_stamp_ns:return
        self.costmap=GridClearance(msg.info.width,msg.info.height,msg.info.resolution,
                                  (msg.info.origin.position.x,msg.info.origin.position.y),msg.data)
        self.costmap_stamp_ns=source

    def on_camera(self,msg):
        if not valid_rgb_image(msg):return
        age_ns=self.get_clock().now().nanoseconds-stamp_ns(msg.header.stamp)
        if not -50_000_000<=age_ns<=500_000_000:return
        a=np.frombuffer(msg.data,np.uint8).reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3)
        self.camera_frame=cv2.cvtColor(a,cv2.COLOR_RGB2BGR) if msg.encoding=='rgb8' else a.copy()

    def observe(self,name,now,moving=False):
        self.observation_align=not moving
        if self.observation_start is None:
            error=wrap(OBSERVATION_HEADING[name]-self.pose[2])
            if abs(error)>.025:
                if moving and abs(error)>.06:return False
            if abs(error)>.025 and not moving:
                # Finish the small body turn while the physical pan starts moving.
                # Evidence is still gated by the encoder and only starts afterwards.
                pan=OBSERVATIONS[name][0][1] if abs(error)<.20 else 0.0
                self.pan_pub.publish(Float64(data=pan));self.publish_command(0.0,self.heading_pid.calculate(error))
                self.log_status('ALIGN_CAMERA '+name);return False
        if not moving:self.publish_command()
        view,angle=OBSERVATIONS[name][self.observation_index]
        elapsed=(now-self.observation_start).nanoseconds/1e9 if self.observation_start else 0.
        feedback=self.observation_feedback
        fresh=self.feedback_update is not None and (now-self.feedback_update).nanoseconds<700_000_000
        person_view=not view.startswith('plate_')
        matching=(fresh and feedback.get('view')==view and
                  feedback.get('attempt')==self.observation_attempt)
        ready=bool(matching and feedback.get('ready',False) and
                   (not person_view or (self.observation_sample==1 and feedback.get('sample')==1 and
                                        person_evidence_ready(feedback))))
        if not ready:
            attempt=2 if elapsed>=10.5 else (1 if elapsed>=6.5 else 0)
            if attempt!=self.observation_attempt:
                self.observation_attempt=attempt;self.observation_sample=0;self.sample_start=now
                matching=False
        if self.sample_start is None:self.sample_start=now
        # Advance only after the encoder-aligned first angle has real stable frames.
        if (person_view and self.observation_sample==0 and matching and
            feedback.get('sample_frames',{}).get('0',0)>=5):
            self.observation_sample=1;self.sample_start=now
        angle += (0., .08, -.08)[self.observation_attempt]
        if person_view and self.observation_sample:angle+=.08
        self.pan_pub.publish(Float64(data=angle))
        if self.observation_start is None:self.observation_start=now
        event={'run_id':self.run_id,'lap_id':self.lap_id,'view':view,'route_point':name,'pan_rad':angle,'pose':self.pose,'pose_source_ns':getattr(self,'pose_source_ns',None),'stamp_ns':now.nanoseconds,'moving_capture':moving,'attempt':self.observation_attempt,'sample':self.observation_sample}
        self.observation_pub.publish(String(data=json.dumps(event)))
        self.log_status(('OBSERVE ' if self.observation_attempt==0 else 'RETRY_OBSERVE ')+view)
        # Stable per-view evidence permits an early finish; uncertain views retain retries.
        # Plate feedback already requires two high-confidence OCR votes,
        # six frames, 1.8 s of observation and a 0.8 s quiet evidence window.
        # Do not add a second fixed 2.5 s wait after those checks succeed.
        # Both views already enforce dwell, stable identities and quiet evidence;
        # people additionally require five + three frames at two actual pan angles.
        minimum_s=1.8
        if (now-self.observation_start).nanoseconds<minimum_s*1e9:return False
        if self.camera_frame is None:return False
        if not ready and elapsed<15.0:return False
        # A quiet detector result far upstream is not coverage of the intended
        # view. Continue sampling until the chassis reaches the observation
        # neighbourhood, so near-edge people can enter the real camera image.
        if moving and person_view and math.dist(self.pose[:2],WAYPOINTS[self.path_end_index]['position'])>.10:
            return False
        outcome={**event,'confirmed':bool(ready),'elapsed_s':round(elapsed,2),'feedback':feedback if fresh else {'reason':'stale or missing perception feedback'}}
        self.observation_outcomes.append(outcome)
        if not ready:self.get_logger().warning('UNCONFIRMED_VIEW '+view+' after bounded retries')
        self.inspection_pub.publish(String(data=json.dumps(outcome)))
        out=Path('log/inspection');out.mkdir(parents=True,exist_ok=True)
        cv2.imwrite(str(out/(view+'.png')),self.camera_frame)
        archive=out/self.run_id/f'lap-{self.lap_id:03d}';archive.mkdir(parents=True,exist_ok=True)
        cv2.imwrite(str(archive/(view+'.png')),self.camera_frame)
        self.saved_views.append(event)
        (out/'views.json').write_text(json.dumps(self.saved_views,indent=2)+'\n')
        (archive/'views.json').write_text(json.dumps(self.saved_views,indent=2)+'\n')
        (out/'outcomes.json').write_text(json.dumps(self.observation_outcomes,indent=2)+'\n')
        (archive/'outcomes.json').write_text(json.dumps(self.observation_outcomes,indent=2)+'\n')
        if not ready:
            self.fault='FAULT inspection view unconfirmed after bounded retries: '+view
            self.log_status(self.fault)
            return False
        self.observation_index+=1;self.observation_start=None
        self.observation_attempt=0
        self.observation_sample=0;self.sample_start=None
        if self.observation_index<len(OBSERVATIONS[name]):return False
        self.observation_index=0;self.observation_align=False;self.pan_pub.publish(Float64(data=0.0))
        self.observation_pub.publish(String(data=json.dumps({'view':'transit'})))
        return True

    def on_perception(self,msg):
        try:data=json.loads(msg.data)
        except ValueError:return
        if (data.get('run_id'),data.get('lap_id'))!=(self.run_id,self.lap_id):return
        counts=data.get('counts');plates=data.get('plates')
        self.recognition_counts=counts.copy() if isinstance(counts,dict) else {}
        self.recognition_plates=list(plates) if isinstance(plates,dict) else []
        self.observation_feedback=data.get('observation_feedback',{})
        self.feedback_update=self.get_clock().now()
        self.perception_update=self.feedback_update
        self.perception_stamp=data.get('stamp')

    def on_odom(self, msg):
        if not valid_odometry(msg):return
        source_ns=stamp_ns(msg.header.stamp);age_ns=self.get_clock().now().nanoseconds-source_ns
        if not -50_000_000<=age_ns<=150_000_000:return
        if self.pose_source_ns is not None and source_ns<self.pose_source_ns:return
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        roll = math.atan2(2 * (q.w*q.x + q.y*q.z), 1 - 2 * (q.x*q.x + q.y*q.y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w*q.y - q.z*q.x))))
        self.tilt = max(abs(roll), abs(pitch))
        if self.pose is not None and not self.completed:
            step = math.hypot(p.x - self.pose[0], p.y - self.pose[1])
            if step < .15:  # Do not count teleports as driving.
                self.distance_travelled += step
        self.pose = (p.x, p.y, yaw_from_quaternion(q))
        self.pose_update = self.get_clock().now()
        self.pose_source_time=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        self.pose_source_ns=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec

    def restart(self, request, response):
        fresh = self.pose_update is not None and (self.get_clock().now()-self.pose_update).nanoseconds < 700_000_000
        if not fresh or not self.completed or self.pose is None or not in_start_bay(body_corners(*self.pose)):
            response.success = False
            response.message = 'Restart requires a live simulation and a completed lap at the start area.'
            return response
        self.index = 0
        self.start_alignment=True
        self.blocked_since=None;self.clear_scans=0;self.recovery_count=0
        self.cancel_path_request()
        self.path_retry_index=None;self.path_attempts=0
        self.path_retry_deadline=None;self.path_retry_grid_ns=None
        self.path_index=-1;self.path=None;self.path_pending=False
        self.completed = False
        self.distance_travelled = 0.0
        self.green_streak = 0
        self.waiting_for_green = False
        self.signal_gate.reset()
        self.crossing_gate=None
        self.signal_context = None
        self.motion_probe = None
        self.fault = None
        self.lap_id += 1
        self.observation_start = None
        self.observation_align = False
        self.observation_index = 0
        self.saved_views = []
        self.observation_outcomes=[]
        self.observation_feedback={}
        self.recognition_counts={};self.recognition_plates=[]
        self.feedback_update=None
        self.observation_attempt=0
        self.observation_sample=0;self.sample_start=None
        self.last_log = ''
        self.inspection_done_index=None;self.last_motion_pose=None
        for controller in (self.speed_pid, self.heading_pid):
            controller.integral = controller.previous = 0.0
        out = Path('log/inspection');out.mkdir(parents=True,exist_ok=True)
        (out/'views.json').write_text('[]\n')
        (out/'outcomes.json').write_text('[]\n')
        self.pan_pub.publish(Float64(data=0.0))
        self.observation_pub.publish(String(data=json.dumps({'view':'transit'})))
        self.publish_lap('started')
        self.log_status(f'START lap={self.lap_id}')
        response.success = True
        response.message = 'Starting another physical lap; Gazebo window stays open.'
        return response

    def on_quality(self,msg):
        try:self.localization_quality=json.loads(msg.data)
        except ValueError:return
        self.quality_update=self.get_clock().now()

    def on_scan(self, msg):
        now=self.get_clock().now();source=stamp_ns(msg.header.stamp)
        if (not valid_laser_scan(msg) or source is None or
                not 0 <= now.nanoseconds-source <= 500_000_000):
            self.clear_scans=0;self.scan_update=None
            return
        if self.scan_source_ns is not None and source <= self.scan_source_ns:
            return
        self.scan_source_ns=source;self.scan=msg;self.scan_update=now
        self.clear_scans=0 if self.front_blocked() else min(5,self.clear_scans+1)

    def fresh_signal(self,now):
        return (self.light_update is not None and self.light_source_ns is not None and
                0 <= (now-self.light_update).nanoseconds <= 500_000_000 and
                0 <= now.nanoseconds-self.light_source_ns <= 500_000_000)

    def update_signal_context(self):
        name = ROUTE[self.index][0] if not self.completed and self.index < len(ROUTE) else ''
        context = ('top' if name in ('top_approach', 'top_stop') else
                   'lower' if name in ('lower_approach', 'lower_stop') else None)
        if context != self.signal_context:
            self.signal_gate.reset()
            self.signal_context = context
        return context

    def on_stamped_light(self,msg):
        try:
            data=json.loads(msg.data);source=data['source_stamp_ns'];colour=data['colour']
            if type(source) is not int or colour not in ('red','green','yellow','unknown'):return
        except (ValueError,KeyError,TypeError):return
        now=self.get_clock().now();age_ns=now.nanoseconds-source
        if not 0<=age_ns<=500_000_000:return
        if self.light_source_ns is not None and source<=self.light_source_ns:return
        self.light_source_ns=source;self.light_state=colour;self.light_update=now
        self.phase_clock.observe(colour,source/1e9)
        if self.update_signal_context() is not None:
            self.signal_gate.observe(colour,source/1e9)
        if self.crossing_gate is not None:
            self.crossing_gate.observe(colour,source/1e9)

    def on_light(self, msg):
        self.light_state = msg.data
        self.light_update = self.get_clock().now()
        self.light_source_ns=self.light_update.nanoseconds
        self.phase_clock.observe(msg.data, self.light_update.nanoseconds / 1e9)
        if self.update_signal_context() is not None:
            self.signal_gate.observe(msg.data, self.light_update.nanoseconds / 1e9)
        if self.crossing_gate is not None:
            self.crossing_gate.observe(msg.data,self.light_update.nanoseconds/1e9)

    def publish_command(self, linear=0.0, angular=0.0):
        self.command_guard=None
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        if self.pose and linear>0:
            x,y,yaw=self.pose; points=footprint(x,y,yaw)
            before_top=abs(y-1.7)<.22 and .30<=min(p[0] for p in points) and x<.58 and abs(wrap(yaw-math.pi))<.5
            before_lower=abs(x+.1)<.22 and -.51<=min(p[1] for p in points) and y<-.20 and abs(wrap(yaw+math.pi/2))<.5
            if before_top or before_lower:
                now=self.get_clock().now();fresh=self.fresh_signal(now)
                authorized=(self.crossing_gate is None or self.crossing_gate.ready_at(now.nanoseconds/1e9))
                if not fresh or self.light_state!='green' or not authorized:
                    linear=angular=0.0
                    self.command_guard='GUARD signal stale, changed or crossing budget exhausted before stop line'
                    self.motion_probe=None
                    gate=self.crossing_gate or self.signal_gate
                    remaining=gate.remaining_at(now.nanoseconds/1e9)
                    if hasattr(self,'signal_hold_pub'):
                        self.signal_hold_pub.publish(String(data=json.dumps({
                            'run_id':self.run_id,'lap_id':self.lap_id,'stamp':now.nanoseconds/1e9,
                            'route_point':ROUTE[self.index][0],'location':'approach',
                            'reason':signal_hold_reason(fresh,self.light_state,authorized,remaining,gate.crossing_budget),
                            'colour':self.light_state,'remaining_lower_s':remaining,
                            'source_age_s':None if self.light_source_ns is None else (now.nanoseconds-self.light_source_ns)/1e9})))
            elif self.crossing_gate is not None:
                self.crossing_gate=None
        road_recovery=False
        if self.pose and (linear or angular):
            for dt in (.1,.3,.5):
                if not clear_of_posts(*predict(*self.pose,linear,angular,dt),self.static_posts,margin=.025):
                    self.command_guard='GUARD physical signal post would intersect footprint'
                    self.motion_probe=None;linear=angular=0.0
                    now=self.get_clock().now().nanoseconds/1e9
                    if now-self.post_replan_time>1.0:
                        self.post_replan_time=now;self.path_index=-1;self.path=None
                    break
                if not safe_pose(*predict(*self.pose,linear,angular,dt)):
                    step=None
                    if self.index<len(ROUTE) and not ROUTE[self.index][3]:
                        step=corridor_recovery(self.pose,ROUTE[self.index][1:3],self.static_posts)
                    if step is not None:
                        linear,angular=step;road_recovery=True
                        self.command_guard='RECOVER_ROAD constrained forward step within corridor'
                        stamp=self.get_clock().now().nanoseconds/1e9
                        if self.road_recovery_started is None:
                            self.road_recovery_started=stamp;self.road_recovery_origin=self.pose[:2]
                        if stamp-self.road_recovery_started>8.:
                            linear=angular=0.;self.fault='FAULT road recovery exceeded eight seconds'
                            self.command_guard=self.fault
                    else:
                        self.command_guard="GUARD footprint would cross road paint"
                        self.motion_probe=None;linear=angular=0.0
                    break
        if (not road_recovery and self.command_guard is None and (linear or angular) and
            self.road_recovery_origin is not None and math.hypot(self.pose[0]-self.road_recovery_origin[0],self.pose[1]-self.road_recovery_origin[1])>.04):
            self.road_recovery_started=None;self.road_recovery_origin=None
        cmd.twist.linear.x = float(linear)
        cmd.twist.angular.z = float(angular)
        self.published_command=(float(linear),float(angular))
        self.command_pub.publish(cmd)

    def log_status(self, value):
        if value != self.last_log:
            self.get_logger().info(value)
            self.last_log = value
        msg = String()
        msg.data = value
        self.status_pub.publish(msg)

    def path_heading_error(self):
        if self.path is None or not self.path.poses:return None
        x,y,yaw=self.pose
        points=[(p.pose.position.x,p.pose.position.y) for p in self.path.poses]
        error,self.path_curvature,self.curve_progress=curve_target(points,self.pose,self.curve_progress,self.path_goal_yaw,
            terminal_tolerance=WAYPOINTS[getattr(self,'path_end_index',self.index)]['tolerance'])
        self.path_points=points;self.path_nearest=self.curve_progress
        self.path_segment_distance=math.dist((x,y),points[-1])
        a,b=points[self.curve_progress],points[self.curve_progress+1]
        length=math.dist(a,b);dx,dy=b[0]-a[0],b[1]-a[1]
        along=max(0.,min(length,((x-a[0])*dx+(y-a[1])*dy)/max(length,1e-9)))
        self.path_remaining_arc=length-along+sum(math.dist(a,b)for a,b in zip(points[self.curve_progress+1:],points[self.curve_progress+2:]))
        return error

    def front_blocked(self):
        if self.scan is None or not self.scan.ranges:
            return False
        step = self.scan.angle_increment
        return any(
            math.isfinite(r) and .04 < r < .28
            for i, r in enumerate(self.scan.ranges)
            if abs(wrap(self.scan.angle_min + i * step)) < .35
        )

    def tick(self):
        now = self.get_clock().now()
        self.update_signal_context()
        if not self.completed:self.check_path_deadline(now)
        if self.tilt > .35:
            self.fault = 'FAULT chassis tilted beyond 20 degrees'
        if self.fault:
            self.publish_command()
            self.log_status(self.fault)
            return
        # Completion is a latched stopped state. Later sensor dropout must
        # not replace it with a transient moving-lap recovery status.
        if self.completed:
            self.publish_command()
            self.motion_probe = None
            self.log_status(f'COMPLETE single lap; travelled={self.distance_travelled:.2f} m')
            return
        if (self.pose is None or (now - self.pose_update).nanoseconds > 700_000_000 or
                self.pose_source_ns is None or now.nanoseconds-self.pose_source_ns>150_000_000):
            self.publish_command()
            self.motion_probe = None
            self.log_status('WAIT_FRESH_LOCALIZATION')
            return
        if self.require_vision and (self.perception_update is None or
                (now-self.perception_update).nanoseconds>800_000_000 or
                self.perception_stamp is None or now.nanoseconds/1e9-self.perception_stamp>.5):
            self.publish_command();self.motion_probe=None
            self.log_status('WAIT_FRESH_PERCEPTION');return
        if (self.scan_update is None or (now - self.scan_update).nanoseconds > 700_000_000 or
                self.scan_source_ns is None or not 0 <= now.nanoseconds-self.scan_source_ns <= 500_000_000):
            self.publish_command()
            self.motion_probe = None
            self.log_status('WAIT_FRESH_SCAN')
            return
        quality_fresh=self.quality_update is not None and (now-self.quality_update).nanoseconds<700_000_000
        if not quality_fresh or self.localization_quality.get('state')=='lost':
            self.publish_command();self.motion_probe=None
            # A stale quality packet can arrive just before a fresh wheel pose.
            # Stop the chassis, but do not interrupt a settled observation pan.
            # Recenter only for actual distance-without-registration loss.
            if not quality_fresh or self.localization_quality.get('reason')=='source or receipt freshness':
                self.log_status('RECOVER_LOCALIZATION wait for fresh source quality; retain observation pan')
                return
            self.pan_pub.publish(Float64(data=0.0))
            self.log_status('RECOVER_LOCALIZATION wait for fresh road-paint registration')
            return
        if getattr(self,'start_alignment',False):
            error=wrap(START[2]-self.pose[2])
            if abs(error)>.025:
                fresh=self.costmap_stamp_ns is not None and 0<=now.nanoseconds-self.costmap_stamp_ns<=2_000_000_000
                if not fresh or not pivot_clear(self.pose[:2],self.costmap,self.static_posts):
                    self.publish_command();self.log_status('START_HOLD waiting for safe departure orientation');return
                self.publish_command(0.,self.heading_pid.calculate(error))
                self.log_status(self.command_guard or 'ALIGN_START in configured bay');return
            self.start_alignment=False
        name, tx, ty, stop = ROUTE[self.index]
        x, y, yaw = self.pose
        dist = math.hypot(tx - x, ty - y)
        previous=getattr(self,'last_motion_pose',None) or self.pose
        self.last_motion_pose=self.pose
        end_index=getattr(self,'path_end_index',self.index)
        rolling=self.path_index==self.index and self.path is not None and self.index<end_index
        inspection_index=end_index if self.path_index==self.index and self.path is not None else self.index
        inspection_point=WAYPOINTS[inspection_index]
        inspection_distance=math.dist(self.pose[:2],inspection_point['position'])
        inspection_zone=.25 if inspection_point['views'] and not inspection_point['views'][0][0].startswith('plate_') else .35
        inspection_pending=bool(inspection_point['views']) and getattr(self,'inspection_done_index',None)!=inspection_index
        if inspection_point['views'] and inspection_distance<.4 and hasattr(self,'prefetch_next'):
            self.prefetch_next(inspection_index,now)
        if (inspection_pending and inspection_distance<inspection_zone and inspection_distance>inspection_point['tolerance'] and
                abs(wrap(inspection_point['heading']-yaw))<.06 and self.localization_quality.get('state')=='tracking'):
            if self.observe(inspection_point['name'],now,moving=True):
                self.inspection_done_index=inspection_index;inspection_pending=False
        if rolling and passage(previous,self.pose,WAYPOINTS[self.index]):
            self.log_status(f'REACHED {name} ({self.index + 1}/{len(ROUTE)}) estimated=({x:.3f},{y:.3f}); continuous passage')
            self.index+=1;self.path_index=self.index
            self.publish_command(*getattr(self,'published_command',(0.,0.)))
            return
        if rolling and ((x-tx)*math.cos(WAYPOINTS[self.index]['heading'])+
                        (y-ty)*math.sin(WAYPOINTS[self.index]['heading']))>.08:
            self.fault='FAULT missed continuous transit passage'
            self.publish_command();self.log_status(self.fault);return
        # Avoid a last-centimetre heading flip toward the waypoint's lateral
        # offset: the camera must keep facing the gantry while stopped.
        tolerance=WAYPOINTS[self.index]['tolerance']
        goal_yaw=getattr(self,'path_goal_yaw',None) if self.path_index==self.index and not rolling else None
        if goal_yaw is not None and getattr(self,'path_curved',False) and dist>=tolerance:
            # The tangent extension is a steering target, not permission to
            # drive indefinitely past a missed goal circle.
            passed=(x-tx)*math.cos(goal_yaw)+(y-ty)*math.sin(goal_yaw)
            if dist<.20 and passed>.025:
                key=(self.lap_id,self.index)
                if getattr(self,'arrival_replan_key',None)!=key:
                    self.arrival_replan_key=key;self.arrival_replans=0
                self.arrival_replans+=1
                self.publish_command();self.motion_probe=None;self.path_index=-1;self.path=None
                if self.arrival_replans>=3:
                    self.fault='FAULT terminal goal missed after three checked arrivals'
                self.log_status(self.fault or 'ARRIVAL_HOLD missed goal; replan checked path');return
        moving_terminal=False
        if dist<tolerance and goal_yaw is not None and abs(wrap(goal_yaw-yaw))>.025:
            fresh=self.costmap_stamp_ns is not None and 0<=now.nanoseconds-self.costmap_stamp_ns<=2_000_000_000
            key=(self.lap_id,self.index)
            if getattr(self,'terminal_tracking_key',None)!=key:
                self.terminal_tracking_key=key;self.terminal_tracking_started=now.nanoseconds/1e9
            elapsed=now.nanoseconds/1e9-self.terminal_tracking_started
            terminal_speed=terminal_curvature=0.
            if self.path is not None and getattr(self,'path_curved',False):
                self.path_heading_error()
                terminal_speed,_=path_velocity(self.path_points,self.path_nearest,self.path_curvature,
                    self.path_remaining_arc,tolerance=tolerance,settings=TASK['motion'])
                terminal_speed=min(.06,terminal_speed,self.path_velocity_envelope['speed_m_s'][self.path_nearest])
                terminal_curvature=self.path_curvature
            moving_terminal=(fresh and self.path is not None and getattr(self,'path_curved',False) and
                self.localization_quality.get('state')=='tracking' and
                rolling_terminal_allowed(self.pose,(tx,ty),goal_yaw,tolerance,self.costmap,self.static_posts,elapsed,
                    speed=terminal_speed,curvature=terminal_curvature))
            if not moving_terminal:
                if not fresh or not pivot_clear(self.pose[:2],self.costmap,self.static_posts):
                    self.publish_command();self.path_index=-1;self.path=None;self.log_status('ARRIVAL_HOLD replan safe final heading');return
                # Goal-pose feedback has its own convergence rate; pursuit's
                # slower heading gain can leave a tiny correction stopped for
                # several ticks. Keep the same angular and slew limits.
                self.publish_command(0.,max(-TASK['motion']['angular_speed'],min(
                    TASK['motion']['angular_speed'],3.0*wrap(goal_yaw-yaw))))
                self.log_status(self.command_guard or 'ALIGN_ARRIVAL '+name);return
        observing=name in OBSERVATIONS and self.observation_align
        if (dist < tolerance and not moving_terminal and not rolling) or observing:
            self.motion_probe = None
            if name in OBSERVATIONS and getattr(self,'inspection_done_index',None)!=self.index:
                if not self.observe(name,now):return
                self.inspection_done_index=self.index
            if stop:
                self.publish_command()
                if not self.waiting_for_green:
                    self.waiting_for_green = True
                    self.green_streak = 0
                recent = self.fresh_signal(now)
                if self.light_state != 'green' or not recent:
                    self.green_streak = 0
                if not recent:self.signal_gate.invalidate()
                if not self.signal_gate.ready_at(now.nanoseconds/1e9) and self.require_vision:
                    remaining=self.signal_gate.remaining_at(now.nanoseconds/1e9)
                    if hasattr(self,'signal_hold_pub'):self.signal_hold_pub.publish(String(data=json.dumps({
                        'run_id':self.run_id,'lap_id':self.lap_id,'stamp':now.nanoseconds/1e9,
                        'route_point':name,'location':'stop','colour':self.light_state,
                        'reason':signal_hold_reason(recent,self.light_state,False,remaining,self.signal_gate.crossing_budget),
                        'remaining_lower_s':remaining,
                        'source_age_s':None if self.light_source_ns is None else (now.nanoseconds-self.light_source_ns)/1e9})))
                    self.log_status(f'STOP {name}: light={self.light_state}; remaining_lower={remaining}; visual_edges={self.phase_clock.edges}')
                    return
                self.green_streak = 0
                self.waiting_for_green = False
                self.signal_decisions.publish(String(data=json.dumps({
                    'run_id':self.run_id,'lap_id':self.lap_id,'gate':name,
                    'stamp':now.nanoseconds/1e9,'colour':self.light_state,
                    'remaining_lower_s':self.signal_gate.remaining_at(now.nanoseconds/1e9),
                    'crossing_budget_s':self.signal_gate.crossing_budget,
                    'camera_edge_count':self.phase_clock.edges,
                    'camera_anchor_interval':self.phase_clock.anchor,
                    'source':'Camera-observed phase edges and fresh green; no simulator state'})))
                self.crossing_gate=self.signal_gate
                self.signal_gate=FreshGreenGate(self.phase_clock,self.crossing_gate.crossing_budget)
            self.pan_pub.publish(Float64(data=0.0))
            self.log_status(f'REACHED {name} ({self.index + 1}/{len(ROUTE)}) estimated=({x:.3f},{y:.3f})')
            self.index += 1
            if self.index<len(ROUTE) and name in OBSERVATIONS and hasattr(self,'take_prefetched_path'):
                self.take_prefetched_path(now)
            if self.index == len(ROUTE):
                required=[view for views in OBSERVATIONS.values() for view,angle in views]
                if not inspection_complete(self.observation_outcomes,required,self.run_id,self.lap_id):
                    self.fault='FAULT incomplete inspection; required views are missing or unconfirmed'
                elif not recognition_complete(self.recognition_counts,self.recognition_plates,self.expected_people,self.expected_plates):
                    self.fault='FAULT incomplete inspection; measured people or plate totals do not meet the task'
                elif self.distance_travelled < 8.0:
                    self.fault = 'FAULT insufficient physical travel for a complete lap'
                else:
                    self.completed = True
                    self.publish_lap('completed')
            return
        if self.front_blocked():
            if self.blocked_since is None:self.blocked_since=now
            self.motion_probe=None
            error=self.path_heading_error() if self.path_index==self.index else None
            nearest=min((r for r in self.scan.ranges if math.isfinite(r) and .04<r),default=0.)
            # Turning sweeps a 0.158 m radius. Only rotate with >0.19 m all-around clearance.
            if error is not None and abs(error)>.15 and nearest>.19:
                self.publish_command(0.,self.heading_pid.calculate(error))
                self.log_status('RECOVER_OBSTACLE align within lane');return
            self.publish_command()
            duration=(now-self.blocked_since).nanoseconds/1e9
            self.log_status(f'BLOCKED before {name}' if duration<30 else f'BLOCKED_TIMEOUT before {name}; waiting for permitted route to clear')
            return
        if self.blocked_since is not None:
            self.publish_command();self.motion_probe=None
            if self.clear_scans<5:
                self.log_status('RECOVER_OBSTACLE confirming clear scans');return
            self.blocked_since=None;self.recovery_count+=1
            self.path_index=-1;self.path=None
            self.log_status('RECOVER_OBSTACLE replan permitted route');return
        if self.path_index!=self.index:
            self.path=None;self.request_path(tx,ty);self.publish_command();self.log_status('PLAN '+name);return
        if self.path_pending or self.path is None:
            self.publish_command();self.log_status('WAIT_NAV2_PATH');return
        error = self.path_heading_error()
        angular = self.heading_pid.calculate(error)
        limits=None
        if getattr(self,'path_align',False):
            self.motion_probe=None
            if now.nanoseconds/1e9<self.path_settle_until:
                self.publish_command();self.log_status('SETTLE_TURN '+name);return
            if abs(error)>.035:
                self.publish_command(0.,angular);self.log_status(self.command_guard or 'ALIGN_ROUTE '+name);return
            self.path_align=False
        if getattr(self,'path_curved',False) and abs(error)>.55:
            fresh=self.costmap_stamp_ns is not None and 0<=now.nanoseconds-self.costmap_stamp_ns<=2_000_000_000
            if not fresh or not pivot_clear(self.pose[:2],self.costmap,self.static_posts):
                self.publish_command();self.path_index=-1;self.path=None
                self.log_status('CURVE_HOLD replan after tracking deviation');return
            self.publish_command(0.,angular);self.log_status('ALIGN_ROUTE '+name);return
        linear,limits=path_velocity(self.path_points,self.path_nearest,self.path_curvature,self.path_remaining_arc,tolerance=WAYPOINTS[end_index]['tolerance'],settings=TASK['motion'])
        if inspection_pending and inspection_distance<inspection_zone:
            linear=min(linear,.06)
        # Extra speed only on straight, well-localized travel, away from stop approaches.
        if ('approach' in name or stop or self.localization_quality.get('state')!='tracking'):
            linear=min(linear,.14 if self.localization_quality.get('state')=='degraded' else .18)
        # A fresh, camera-anchored green can permit normal straight speed.
        # The release gate and whole-body budget remain unchanged.
        if 'approach' in name:
            fresh=self.fresh_signal(now)
            remaining=self.phase_clock.remaining_lower(now.nanoseconds/1e9)
            enough=(fresh and self.light_state=='green' and remaining is not None and
                    remaining>=max(0.,dist-.05)/.18+self.signal_gate.crossing_budget)
            if enough and abs(error)<.06 and self.localization_quality.get('state')=='tracking':
                linear=min(limits['selected_before_semantic_caps'],.26)
            self.approach_decisions.publish(String(data=json.dumps({'run_id':self.run_id,'lap_id':self.lap_id,'route_point':name,'stamp':now.nanoseconds/1e9,'remaining_lower_s':remaining,'green_sufficient_for_approach_and_body':enough,'commanded_linear_m_s':linear,'source':'Fresh camera colour and camera-observed phase anchor'})))
        linear=min(linear,self.path_velocity_envelope['speed_m_s'][self.path_nearest])
        if moving_terminal:linear=min(linear,.06)
        angular=linear*self.path_curvature
        if self.motion_probe is None:
            self.motion_probe = (now, x, y, yaw)
        elif (now - self.motion_probe[0]).nanoseconds > 4_000_000_000:
            _, px, py, pyaw = self.motion_probe
            if math.hypot(x-px, y-py) < .015 and abs(wrap(yaw-pyaw)) < .04:
                self.fault = 'FAULT commands present but chassis has not physically moved'
                self.publish_command()
                self.log_status(self.fault)
                return
            self.motion_probe = (now, x, y, yaw)
        # Prepare the next semantic view even when a transit waypoint precedes
        # it (e.g. the two coincident arrival/observation points). Actual votes
        # still require fresh images and a measured settled camera angle.
        # Navigation keeps the real wheel pose; side views do not correct ground paint.
        prepan=(inspection_pending and inspection_distance<max(.4,inspection_zone) and
                abs(wrap(inspection_point['heading']-yaw))<.5 and
                self.localization_quality.get('state')=='tracking')
        if self.observation_start is None:
            self.pan_pub.publish(Float64(data=inspection_point['views'][0][1] if prepan else 0.0))
        stamp=now.nanoseconds/1e9
        dt=stamp-getattr(self,'last_tracking_command_stamp',stamp-.1)
        linear,angular=coupled_command(linear,self.path_curvature,getattr(self,'published_command',(0.,0.)),dt,TASK['motion'])
        self.last_tracking_command_stamp=stamp
        self.publish_command(linear, angular)
        if hasattr(self,'control_trace_pub'):
            self.control_trace_pub.publish(String(data=json.dumps({'run_id':self.run_id,'lap_id':self.lap_id,
                'route':name,'stamp':now.nanoseconds/1e9,'pose':self.pose,'curved':getattr(self,'path_curved',False),
                'linear':linear,'angular':angular,'published_command':self.published_command,
                'localization_quality':self.localization_quality.get('state'),
                'terminal_tracking':moving_terminal,
                'pose_age_s':None if self.pose_source_ns is None else (now.nanoseconds-self.pose_source_ns)/1e9,
                'heading_error':error,'limits':limits,
                'guard':self.command_guard})))
        active_view=(OBSERVATIONS[inspection_point['name']][self.observation_index][0]
                     if self.observation_start is not None and inspection_point['views'] else None)
        self.log_status(self.command_guard or ('OBSERVE '+active_view if active_view else f'GO {name} ({self.index + 1}/{len(ROUTE)})'))


def main():
    rclpy.init()
    node = Patrol()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.publish_command()
        node.destroy_node()
        rclpy.try_shutdown()
