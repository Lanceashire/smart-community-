"""Estimated-pose-followed simulation patrol with signal stop gates.

This deliberately small first-run controller feeds Sentry26's command manager.
It is not a replacement for SLAM/Nav2 required by a complete semifinal entry.
"""

import math
import json
from pathlib import Path
import cv2
import numpy as np

import rclpy
from geometry_msgs.msg import TwistStamped,PoseStamped
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import String, Float64
from std_srvs.srv import Trigger

from community_demo.bounded_pid import BoundedPID
from community_demo.road_rules import safe_pose,predict,footprint


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
    ('lower_south', -.1, -1.82, False), ('bottom_east', 1.1, -1.82, False),
    ('observe_plate_1', 1.1, -1.7, False), ('observe_plate_2', 1.1, -1.1, False),
    ('observe_plate_3', 1.1, -.5, False), ('right_north', 1.1, 1.7, False),
]
OBSERVATION_HEADING={'observe_A_north':math.pi,'observe_A_west':-math.pi/2,'observe_middle':0.,'observe_B_east':-math.pi/2,'observe_plate_1':math.pi/2,'observe_plate_2':math.pi/2,'observe_plate_3':math.pi/2}
OBSERVATIONS = {
    'observe_A_north':[('A_north', math.pi/2)],
    'observe_A_west':[('A_west', math.pi/2)],
    'observe_middle':[('A_south', math.pi/2),('B_north', -math.pi/2)],
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
        self.pan_pub=self.create_publisher(Float64,'/model/patrol_bot/camera_pan',10)
        self.observation_pub=self.create_publisher(String,'/inspection/observation',10)
        self.create_subscription(Image,'/camera/image',self.on_camera,10)
        self.camera_frame=None
        self.observation_start=None
        self.observation_align=False
        self.observation_index=0
        self.saved_views=[]
        self.planner=ActionClient(self,ComputePathToPose,'/compute_path_to_pose')
        self.path=None;self.path_pending=False;self.path_index=-1;self.path_pub=self.create_publisher(__import__('nav_msgs.msg',fromlist=['Path']).Path,'/patrol/path',10)
        self.command_pub = self.create_publisher(TwistStamped, '/cmd_vel_nav', 10)
        self.status_pub = self.create_publisher(String, '/patrol/status', 10)
        # Control consumes camera/wheel localization. Simulator truth is evaluation only.
        self.create_subscription(Odometry, self.get_parameter('pose_topic').value, self.on_odom, 10)
        self.create_service(Trigger, '/patrol/restart', self.restart)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.create_subscription(String, '/perception/traffic_light', self.on_light, 10)
        self.pose = None
        self.pose_update = None
        self.tilt = 0.0
        self.distance_travelled = 0.0
        self.motion_probe = None
        self.fault = None
        self.scan = None
        self.scan_update = None
        self.light_state = 'unknown'
        self.light_update = None
        self.index = 0
        self.green_streak = 0
        self.waiting_for_green = False
        self.completed = False
        self.last_log = ''
        self.speed_pid = BoundedPID(kp=.55, lower=0.0, upper=.18)
        self.heading_pid = BoundedPID(kp=1.8, lower=-.65, upper=.65)
        self.create_timer(.1, self.tick)
        self.get_logger().info('Single-lap route ready; uses camera / wheel localization with separate laser SLAM mapping and camera-confirmed green.')

    def request_path(self,tx,ty):
        if not self.planner.server_is_ready():return
        goal=ComputePathToPose.Goal();goal.goal=PoseStamped();goal.goal.header.frame_id='field';goal.goal.header.stamp=self.get_clock().now().to_msg();goal.goal.pose.position.x=float(tx);goal.goal.pose.position.y=float(ty);goal.goal.pose.orientation.w=1.0;goal.planner_id='GridBased'
        self.path_pending=True;self.path_index=self.index
        future=self.planner.send_goal_async(goal)
        def accepted(f):
            handle=f.result()
            if not handle.accepted:self.path_pending=False;self.fault='FAULT Nav2 rejected planning goal';return
            def result(g):
                self.path_pending=False;path=g.result().result.path
                if not path.poses:self.fault='FAULT Nav2 could not find a drivable path';return
                self.path=path;self.path_pub.publish(path)
            handle.get_result_async().add_done_callback(result)
        future.add_done_callback(accepted)

    def on_camera(self,msg):
        if msg.encoding not in ('rgb8','bgr8'):return
        a=np.frombuffer(msg.data,np.uint8).reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3)
        self.camera_frame=cv2.cvtColor(a,cv2.COLOR_RGB2BGR) if msg.encoding=='rgb8' else a.copy()

    def observe(self,name,now):
        self.observation_align=True
        if self.observation_start is None:
            error=wrap(OBSERVATION_HEADING[name]-self.pose[2])
            if abs(error)>.025:
                self.pan_pub.publish(Float64(data=0.0));self.publish_command(0.0,self.heading_pid.calculate(error))
                self.log_status('ALIGN_CAMERA '+name);return False
        self.publish_command()
        view,angle=OBSERVATIONS[name][self.observation_index]
        self.pan_pub.publish(Float64(data=angle))
        if self.observation_start is None:self.observation_start=now
        event={'view':view,'route_point':name,'pan_rad':angle,'pose':self.pose}
        self.observation_pub.publish(String(data=json.dumps(event)))
        self.log_status('OBSERVE '+view)
        if (now-self.observation_start).nanoseconds<5_500_000_000:return False
        if self.camera_frame is None:return False
        out=Path('log/inspection');out.mkdir(parents=True,exist_ok=True)
        cv2.imwrite(str(out/(view+'.png')),self.camera_frame)
        self.saved_views.append(event)
        (out/'views.json').write_text(json.dumps(self.saved_views,indent=2)+'\n')
        self.observation_index+=1;self.observation_start=None
        if self.observation_index<len(OBSERVATIONS[name]):return False
        self.observation_index=0;self.observation_align=False;self.pan_pub.publish(Float64(data=0.0))
        self.observation_pub.publish(String(data=json.dumps({'view':'transit'})))
        return True

    def on_odom(self, msg):
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

    def restart(self, request, response):
        fresh = self.pose_update is not None and (self.get_clock().now()-self.pose_update).nanoseconds < 700_000_000
        if not fresh or not self.completed or self.pose is None or math.hypot(self.pose[0]-1.1, self.pose[1]-1.7) > .25:
            response.success = False
            response.message = 'Restart requires a live simulation and a completed lap at the start area.'
            return response
        self.index = 0
        self.path_index=-1;self.path=None;self.path_pending=False
        self.completed = False
        self.distance_travelled = 0.0
        self.green_streak = 0
        self.waiting_for_green = False
        self.motion_probe = None
        self.fault = None
        response.success = True
        response.message = 'Starting another physical lap; Gazebo window stays open.'
        return response

    def on_scan(self, msg):
        self.scan = msg
        self.scan_update = self.get_clock().now()

    def on_light(self, msg):
        self.light_state = msg.data
        self.light_update = self.get_clock().now()
        if self.waiting_for_green:
            self.green_streak = self.green_streak + 1 if msg.data == 'green' else 0

    def publish_command(self, linear=0.0, angular=0.0):
        self.command_guard=None
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        if self.pose and linear>0:
            x,y,yaw=self.pose; points=footprint(x,y,yaw)
            before_top=abs(y-1.7)<.22 and .30<=min(p[0] for p in points) and x<.58 and abs(wrap(yaw-math.pi))<.5
            before_lower=abs(x+.1)<.22 and -.51<=min(p[1] for p in points) and -.35<y<-.20 and abs(wrap(yaw+math.pi/2))<.5
            if before_top or before_lower:
                fresh=self.light_update is not None and (self.get_clock().now()-self.light_update).nanoseconds<500_000_000
                if not fresh or self.light_state!='green':
                    linear=angular=0.0
                    self.command_guard='GUARD signal changed or became stale before stop line'
                    self.motion_probe=None
        if self.pose and (linear or angular):
            for dt in (.1,.3,.5):
                if not safe_pose(*predict(*self.pose,linear,angular,dt)):
                    self.command_guard="GUARD footprint would cross road paint"
                    self.motion_probe=None
                    linear=angular=0.0
                    break
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

    def front_blocked(self):
        if self.scan is None or not self.scan.ranges:
            return False
        step = self.scan.angle_increment
        return any(
            math.isfinite(r) and .04 < r < .22
            for i, r in enumerate(self.scan.ranges)
            if abs(wrap(self.scan.angle_min + i * step)) < .35
        )

    def tick(self):
        now = self.get_clock().now()
        if self.pose is None or (now - self.pose_update).nanoseconds > 700_000_000:
            self.publish_command()
            self.motion_probe = None
            self.log_status('WAIT_FRESH_LOCALIZATION')
            return
        if self.tilt > .35:
            self.fault = 'FAULT chassis tilted beyond 20 degrees'
        if self.fault:
            self.publish_command()
            self.log_status(self.fault)
            return
        if self.completed:
            self.publish_command()
            self.log_status(f'COMPLETE single lap; travelled={self.distance_travelled:.2f} m')
            return
        if self.scan_update is None or (now - self.scan_update).nanoseconds > 700_000_000:
            self.publish_command()
            self.motion_probe = None
            self.log_status('WAIT_FRESH_SCAN')
            return
        name, tx, ty, stop = ROUTE[self.index]
        x, y, yaw = self.pose
        dist = math.hypot(tx - x, ty - y)
        # Avoid a last-centimetre heading flip toward the waypoint's lateral
        # offset: the camera must keep facing the gantry while stopped.
        tolerance = .05 if name in ('top_approach', 'top_stop',
                                     'lower_approach', 'lower_stop') else .04
        observing=name in OBSERVATIONS and self.observation_align
        if dist < tolerance or observing:
            self.motion_probe = None
            if name in OBSERVATIONS and not self.observe(name,now):return
            if stop:
                self.publish_command()
                if not self.waiting_for_green:
                    self.waiting_for_green = True
                    self.green_streak = 0
                recent = (self.light_update is not None and
                          (self.get_clock().now() - self.light_update).nanoseconds < 700_000_000)
                if self.light_state != 'green' or not recent:
                    self.green_streak = 0
                if self.green_streak < 5 and self.require_vision:
                    self.log_status(f'STOP {name}: light={self.light_state}')
                    return
                self.green_streak = 0
                self.waiting_for_green = False
            self.pan_pub.publish(Float64(data=0.0))
            self.log_status(f'REACHED {name} ({self.index + 1}/{len(ROUTE)}) estimated=({x:.3f},{y:.3f})')
            self.index += 1
            if self.index == len(ROUTE):
                if self.distance_travelled < 8.0:
                    self.fault = 'FAULT insufficient physical travel for a complete lap'
                else:
                    self.completed = True
            return
        if self.front_blocked():
            self.publish_command()
            self.motion_probe = None
            self.log_status(f'BLOCKED before {name}')
            return
        if self.path_index!=self.index:
            self.path=None;self.request_path(tx,ty);self.publish_command();self.log_status('PLAN '+name);return
        if self.path_pending or self.path is None:
            self.publish_command();self.log_status('WAIT_NAV2_PATH');return
        points=[(p.pose.position.x,p.pose.position.y) for p in self.path.poses]
        nearest=min(range(len(points)),key=lambda i:math.hypot(points[i][0]-x,points[i][1]-y))
        lookahead=nearest
        while lookahead<len(points)-1 and math.hypot(points[lookahead][0]-x,points[lookahead][1]-y)<.14:lookahead+=1
        gx,gy=points[lookahead]
        error = wrap(math.atan2(gy - y, gx - x) - yaw)
        angular = self.heading_pid.calculate(error)
        if abs(error) > .35:
            linear = 0.0
        else:
            linear = max(.06, self.speed_pid.calculate(dist))
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
        self.pan_pub.publish(Float64(data=0.0))
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
