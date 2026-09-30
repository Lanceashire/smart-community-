"""Physical-pose-followed simulation patrol with signal stop gates.

This deliberately small first-run controller feeds Sentry26's command manager.
It is not a replacement for SLAM/Nav2 required by a complete semifinal entry.
"""

import math

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger

from community_demo.bounded_pid import BoundedPID


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


# Centre-line route from the supplied sketch. The two stop waypoints put the
# front of the 0.22 m body before the white stop line (0.16 m nominal margin).
ROUTE = [
    ('top_approach', 0.04, 1.55, False),
    ('top_stop', 0.04, 1.55, True),
    ('top_west', -1.55, 1.55, False),
    ('left_south', -1.55, .50, False),
    ('middle_east', .30, .50, False),
    ('lower_approach', .30, -.84, False),
    ('lower_stop', .30, -.84, True),
    ('lower_south', .30, -1.55, False),
    ('bottom_east', 1.35, -1.55, False),
    ('right_north', 1.35, 1.55, False),
]


class Patrol(Node):
    def __init__(self):
        super().__init__('community_patrol')
        self.declare_parameter('require_vision', True)
        self.require_vision = self.get_parameter('require_vision').value
        self.command_pub = self.create_publisher(TwistStamped, '/cmd_vel_nav', 10)
        self.status_pub = self.create_publisher(String, '/patrol/status', 10)
        # DiffDrive wheel odometry can advance while the body is tipped or stuck.
        # This simulation baseline explicitly uses Gazebo's physical world pose.
        self.create_subscription(Odometry, '/model/patrol_bot/ground_truth', self.on_odom, 10)
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
        self.get_logger().info('Single-lap route ready; uses PHYSICAL world pose and camera-confirmed green.')

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
        if not fresh or not self.completed or self.pose is None or math.hypot(self.pose[0]-1.35, self.pose[1]-1.55) > .25:
            response.success = False
            response.message = 'Restart requires a live simulation and a completed lap at the start area.'
            return response
        self.index = 0
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
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
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
            self.log_status('WAIT_PHYSICAL_POSE')
            return
        if self.tilt > .35:
            self.fault = 'FAULT chassis tilted beyond 20 degrees'
        if self.fault:
            self.publish_command()
            self.log_status(self.fault)
            return
        if self.completed:
            self.publish_command()
            self.log_status(f'COMPLETE physical single lap; travelled={self.distance_travelled:.2f} m')
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
        tolerance = .16 if name in ('top_approach', 'top_stop',
                                     'lower_approach', 'lower_stop') else .10
        if dist < tolerance:
            self.motion_probe = None
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
            self.log_status(f'REACHED {name} ({self.index + 1}/{len(ROUTE)}) physical=({x:.3f},{y:.3f})')
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
        error = wrap(math.atan2(ty - y, tx - x) - yaw)
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
        self.publish_command(linear, angular)
        self.log_status(f'GO {name} ({self.index + 1}/{len(ROUTE)})')


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
