"""Nav2-planned single-lap patrol, camera stop gates and explicit lap lifecycle."""

import math
import json
import uuid
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
from community_demo.motion_profile import bend_limit
from community_demo.sensor_validation import valid_rgb_image,valid_odometry,stamp_ns,valid_laser_scan
from community_demo.inspection_policy import person_evidence_ready,signal_hold_reason
from community_demo.route_path import GridClearance,simplify_path,continuous_lower_path,curve_target,pivot_clear


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


# Centre-line route from the supplied sketch. The two stop waypoints put the
# front of the 0.22 m body before the white stop line (0.16 m nominal margin).
ROUTE = [
    ('top_approach', .52, 1.7, False), ('top_stop', .52, 1.7, True),
    ('observe_A_north', -.74, 1.7, False), ('top_west', -1.7, 1.7, False),
    ('observe_A_west', -1.7, 1.04, False), ('left_south', -1.7, .5, False),
    ('observe_middle', -.90, .5, False), ('middle_east', -.1, .5, False),
    ('observe_B_east', -.1, -.09, False),
    ('lower_approach', -.1, -.29, False), ('lower_stop', -.1, -.29, True),
    # Enter the checked arc earlier, leaving room for the full body to turn
    # continuously below the gantry and return to the east-lane centre.
    ('lower_south', -.1, -1.67, False), ('bottom_east', 1.1, -1.7, False),
    ('observe_plate_1', 1.1, -1.7, False), ('observe_plate_2', 1.1, -1.1, False),
    ('observe_plate_3', 1.1, -.5, False), ('right_north', 1.1, 1.7, False),
]
OBSERVATION_HEADING={'observe_A_north':math.pi,'observe_A_west':-math.pi/2,'observe_middle':0.,'observe_B_east':-math.pi/2,'observe_plate_1':math.pi/2,'observe_plate_2':math.pi/2,'observe_plate_3':math.pi/2}
OBSERVATIONS = {
    'observe_A_north':[('A_north', math.pi/2)],
    'observe_A_west':[('A_west', math.pi/2)],
    'observe_middle':[('A_south', math.pi/2),('B_north', -math.pi/2+.16)],
    'observe_B_east':[('B_east', -math.pi/2)],
    'observe_plate_1':[('plate_1', -math.pi/2)],
    'observe_plate_2':[('plate_2', -math.pi/2)],
    'observe_plate_3':[('plate_3', -math.pi/2)],
}



class Patrol(Node):
    def __init__(self):
        super().__init__('community_patrol')
        self.declare_parameter('require_vision', True)
        self.declare_parameter('pose_topic','/localization/pose')
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
        self.planner=ActionClient(self,ComputePathToPose,'/compute_path_to_pose')
        self.path_request_token=0;self.path_request_deadline=None;self.path_goal_handle=None
        self.path_retry_index=None;self.path_attempts=0
        self.path=None;self.path_pending=False;self.path_index=-1;self.path_pub=self.create_publisher(NavPath,'/patrol/path',10)
        self.raw_path_pub=self.create_publisher(NavPath,'/patrol/raw_path',10)
        self.costmap=None;self.costmap_stamp_ns=None
        self.create_subscription(OccupancyGrid,'/global_costmap/costmap',self.on_costmap,
                                 QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.path_refined=False;self.path_segment=1;self.path_align=False;self.path_settle_until=0.
        self.path_curved=False;self.curve_progress=0;self.path_curvature=0.
        self.command_pub = self.create_publisher(TwistStamped, '/cmd_vel_nav', 10)
        self.status_pub = self.create_publisher(String, '/patrol/status', 10)
        self.signal_decisions = self.create_publisher(String, '/patrol/signal_decision', 10)
        self.signal_hold_pub = self.create_publisher(String, '/patrol/signal_hold', 10)
        self.approach_decisions = self.create_publisher(String, '/patrol/approach_decision', 10)
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
            self.get_logger().warning('Nav2 bounded retry: '+reason)

    def check_path_deadline(self,now):
        if (self.path_pending and self.path_request_deadline is not None and
                now.nanoseconds/1e9 >= self.path_request_deadline):
            self.fail_path_request('five-second request timeout')

    def request_path(self,tx,ty):
        if self.path_retry_index != self.index:
            self.path_retry_index=self.index;self.path_attempts=0
        self.cancel_path_request()
        self.path_attempts += 1
        self.path_pending=True;self.path_index=self.index
        self.path_request_deadline=self.get_clock().now().nanoseconds/1e9+5.
        token=self.path_request_token;route_index=self.index
        if not self.planner.server_is_ready():return
        goal=ComputePathToPose.Goal();goal.goal=PoseStamped()
        goal.goal.header.frame_id='field';goal.goal.header.stamp=self.get_clock().now().to_msg()
        goal.goal.pose.position.x=float(tx);goal.goal.pose.position.y=float(ty)
        goal.goal.pose.orientation.w=1.0;goal.planner_id='GridBased'
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
                        self.path_pending=False;self.path_request_deadline=None;self.path_goal_handle=None
                        self.path_refined=False
                        self.path_curved=False;self.curve_progress=0
                        if hasattr(self,'raw_path_pub'):self.raw_path_pub.publish(path)
                        now_ns=self.get_clock().now().nanoseconds
                        grid=getattr(self,'costmap',None);grid_stamp=getattr(self,'costmap_stamp_ns',None)
                        points=[(p.pose.position.x,p.pose.position.y) for p in path.poses]
                        refined=None;curve=None
                        if ROUTE[self.index][0]=='bottom_east' and grid is not None and grid_stamp is not None and 0<=now_ns-grid_stamp<=2_000_000_000:
                            curve=continuous_lower_path(points,self.pose,grid,self.static_posts)
                            refined=[p[:2] for p in curve] if curve is not None else simplify_path(points,self.pose[:2],grid,self.static_posts)
                            if refined is not None:
                                clean=NavPath();clean.header=path.header
                                for j,(x,y) in enumerate(refined):
                                    p=PoseStamped();p.header=path.header;p.pose.position.x=float(x);p.pose.position.y=float(y);p.pose.orientation.w=1.
                                    if curve is not None:
                                        p.pose.orientation.z=math.sin(curve[j][2]/2);p.pose.orientation.w=math.cos(curve[j][2]/2)
                                    clean.poses.append(p)
                                self.get_logger().info(f'PATH_REFINED {ROUTE[self.index][0]}: {len(points)} -> {len(refined)} points; swept body and live grid checked')
                                path=clean;self.path_refined=True;self.path_segment=1;self.path_curved=curve is not None
                        if ROUTE[self.index][0]=='bottom_east':
                            out=Path('log/paths')/self.run_id;out.mkdir(parents=True,exist_ok=True)
                            evidence={'run_id':self.run_id,'lap_id':self.lap_id,'stamp_ns':now_ns,
                                'pose':self.pose,'pose_source_ns':self.pose_source_ns,
                                'grid_source_ns':grid_stamp,'grid_age_s':None if grid_stamp is None else (now_ns-grid_stamp)/1e9,
                                'raw_points':points,'refined_points':refined,
                                'strategy':'continuous_arc' if curve is not None else 'checked_segments',
                                'curve_poses':curve,
                                'grid':None if grid is None else {'width':grid.width,'height':grid.height,
                                    'resolution':grid.resolution,'origin':grid.origin,'data':list(grid.data)}}
                            (out/f'lap-{self.lap_id:03d}-bottom-{self.path_attempts}.json').write_text(json.dumps(evidence)+'\n')
                        if ROUTE[self.index][0]=='bottom_east' and not self.path_refined:
                            self.fail_path_request('lower junction needs a fresh grid and a whole-body-safe simplified path');return
                        self.path_attempts=0
                        self.path_align=self.path_refined and not self.path_curved;self.path_settle_until=now_ns/1e9+.35
                        self.path=path;self.path_pub.publish(path)
                    except Exception as exc:self.fail_path_request('result exception: '+str(exc))
                handle.get_result_async().add_done_callback(result)
            except Exception as exc:
                if current():self.fail_path_request('goal exception: '+str(exc))
        try:self.planner.send_goal_async(goal).add_done_callback(accepted)
        except Exception as exc:self.fail_path_request('send exception: '+str(exc))

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

    def observe(self,name,now):
        self.observation_align=True
        if self.observation_start is None:
            error=wrap(OBSERVATION_HEADING[name]-self.pose[2])
            if abs(error)>.025:
                # Finish the small body turn while the physical pan starts moving.
                # Evidence is still gated by the encoder and only starts afterwards.
                pan=OBSERVATIONS[name][0][1] if abs(error)<.20 else 0.0
                self.pan_pub.publish(Float64(data=pan));self.publish_command(0.0,self.heading_pid.calculate(error))
                self.log_status('ALIGN_CAMERA '+name);return False
        self.publish_command()
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
        event={'run_id':self.run_id,'lap_id':self.lap_id,'view':view,'route_point':name,'pan_rad':angle,'pose':self.pose,'attempt':self.observation_attempt,'sample':self.observation_sample}
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
        if not fresh or not self.completed or self.pose is None or math.hypot(self.pose[0]-1.1, self.pose[1]-1.7) > .25:
            response.success = False
            response.message = 'Restart requires a live simulation and a completed lap at the start area.'
            return response
        self.index = 0
        self.blocked_since=None;self.clear_scans=0;self.recovery_count=0
        self.cancel_path_request()
        self.path_retry_index=None;self.path_attempts=0
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
        self.feedback_update=None
        self.observation_attempt=0
        self.observation_sample=0;self.sample_start=None
        self.last_log = ''
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
        if getattr(self,'path_curved',False):
            error,self.path_curvature,self.curve_progress=curve_target(points,self.pose,self.curve_progress)
            self.path_points=points;self.path_nearest=self.curve_progress
            self.path_segment_distance=math.dist((x,y),points[-1])
            return error
        if self.path_refined:
            target=self.path_segment
            if target<len(points)-1 and math.dist((x,y),points[target])<.02:
                target+=1;self.path_segment=target;self.path_align=True
                self.path_settle_until=self.get_clock().now().nanoseconds/1e9+.35
            self.path_points=points;self.path_nearest=target-1
            a,b=points[target-1],points[target]
            # Follow a stable segment, not NavFn's centimetre-scale zigzags.
            dx,dy=b[0]-a[0],b[1]-a[1];length=math.hypot(dx,dy)
            along=max(0.,min(length,((x-a[0])*dx+(y-a[1])*dy)/length))
            look=min(length,along+.22)
            gx,gy=a[0]+look*dx/length,a[1]+look*dy/length
            self.path_segment_distance=math.dist((x,y),b)
            if self.path_align:gx,gy=b
            return wrap(math.atan2(gy-y,gx-x)-yaw)
        nearest=min(range(len(points)),key=lambda i:math.hypot(points[i][0]-x,points[i][1]-y))
        self.path_points=points;self.path_nearest=nearest
        lookahead=nearest
        near_post=any(math.hypot(p['x']-x,p['y']-y)<.45 for p in self.static_posts)
        # Short lookahead around a real bend, not every straight pass of a post.
        turning=bend_limit(points,nearest,0.,1.,.24)<.24
        distance=.09 if near_post and turning else .14
        while lookahead<len(points)-1 and math.hypot(points[lookahead][0]-x,points[lookahead][1]-y)<distance:lookahead+=1
        gx,gy=points[lookahead]
        return wrap(math.atan2(gy-y,gx-x)-yaw)

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
        name, tx, ty, stop = ROUTE[self.index]
        x, y, yaw = self.pose
        dist = math.hypot(tx - x, ty - y)
        # Avoid a last-centimetre heading flip toward the waypoint's lateral
        # offset: the camera must keep facing the gantry while stopped.
        tolerance = .05 if name in ('top_approach', 'top_stop',
                                     'lower_approach', 'lower_stop') else .04
        if name=='lower_south':tolerance=.008
        observing=name in OBSERVATIONS and self.observation_align
        if dist < tolerance or observing:
            self.motion_probe = None
            if name in OBSERVATIONS and not self.observe(name,now):return
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
            if self.index == len(ROUTE):
                if self.distance_travelled < 8.0:
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
        if abs(error) > .35 and not getattr(self,'path_curved',False):
            linear = 0.0
        else:
            remaining=min(dist,self.path_segment_distance) if getattr(self,'path_refined',False) else dist
            linear = max(.06, self.speed_pid.calculate(remaining))
            # Extra speed only on straight, well-localized travel, away from stop approaches.
            if abs(error)>.12 or 'approach' in name or stop or self.localization_quality.get('state')!='tracking':
                linear=min(linear,.14 if self.localization_quality.get('state')=='degraded' else .18)
            # A fresh, camera-anchored green can permit normal straight speed.
            # The release gate and whole-body budget remain unchanged.
            if 'approach' in name:
                fresh=self.fresh_signal(now)
                remaining=self.phase_clock.remaining_lower(now.nanoseconds/1e9)
                enough=(fresh and self.light_state=='green' and remaining is not None and
                        remaining>=max(0.,dist-.05)/.18+self.signal_gate.crossing_budget)
                if enough and abs(error)<.06 and self.localization_quality.get('state')=='tracking':
                    linear=max(linear,min(.26,self.speed_pid.calculate(dist)))
                self.approach_decisions.publish(String(data=json.dumps({'run_id':self.run_id,'lap_id':self.lap_id,'route_point':name,'stamp':now.nanoseconds/1e9,'remaining_lower_s':remaining,'green_sufficient_for_approach_and_body':enough,'commanded_linear_m_s':linear,'source':'Fresh camera colour and camera-observed phase anchor'})))
            if getattr(self,'path_curved',False):
                linear=min(linear,.16,.55/max(abs(self.path_curvature),.01))
                angular=linear*self.path_curvature
            else:
                linear=bend_limit(self.path_points,self.path_nearest,error,dist,linear)
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
        # Pre-position only in the last 12 cm of an aligned observation approach.
        # Navigation keeps the real wheel pose; side views do not correct ground paint.
        prepan=(name in OBSERVATIONS and dist<.12 and
                abs(wrap(OBSERVATION_HEADING[name]-yaw))<.035 and
                self.localization_quality.get('state')=='tracking')
        self.pan_pub.publish(Float64(data=OBSERVATIONS[name][0][1] if prepan else 0.0))
        self.publish_command(linear, angular)
        self.log_status(self.command_guard or f'GO {name} ({self.index + 1}/{len(ROUTE)})')


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
