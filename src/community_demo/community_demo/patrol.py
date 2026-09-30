"""One-shot, odometry-followed patrol with signal stop gates.

This deliberately small first-run controller feeds Sentry26's command manager.
It is not a replacement for SLAM/Nav2 required by a complete semifinal entry.
"""

import math

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

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
    ('bottom_east', 1.55, -1.55, False),
    ('right_north', 1.55, 1.55, False),
]


class Patrol(Node):
    def __init__(self):
        super().__init__('community_patrol')
        self.declare_parameter('require_vision', True)
        self.require_vision = self.get_parameter('require_vision').value
        self.command_pub = self.create_publisher(TwistStamped, '/cmd_vel_nav', 10)
        self.status_pub = self.create_publisher(String, '/patrol/status', 10)
        self.create_subscription(Odometry, '/model/patrol_bot/odometry', self.on_odom, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.create_subscription(String, '/perception/traffic_light', self.on_light, 10)
        self.pose = None
        self.scan = None
        self.light_state = 'unknown'
        self.light_update = None
        self.index = 0
        self.green_streak = 0
        self.completed = False
        self.last_log = ''
        self.speed_pid = BoundedPID(kp=.55, lower=0.0, upper=.18)
        self.heading_pid = BoundedPID(kp=1.8, lower=-.65, upper=.65)
        self.create_timer(.1, self.tick)
        self.get_logger().info('Single-lap route ready; requires camera-confirmed green at both stops.')

    def on_odom(self, msg):
        p = msg.pose.pose.position
        # Gazebo DiffDrive odometry starts at (0, 0, 0) even though the model
        # is spawned at (1.65, 1.55, pi) in the 4.2 m world. Compose that
        # fixed spawn transform before comparing against world waypoints.
        self.pose = (1.65 - p.x, 1.55 - p.y,
                     wrap(math.pi + yaw_from_quaternion(msg.pose.pose.orientation)))

    def on_scan(self, msg):
        self.scan = msg

    def on_light(self, msg):
        self.light_state = msg.data
        self.light_update = self.get_clock().now()

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
        n = len(self.scan.ranges)
        step = self.scan.angle_increment
        return any(
            math.isfinite(r) and .04 < r < .22
            for i, r in enumerate(self.scan.ranges)
            if abs(wrap(self.scan.angle_min + i * step)) < .35
        )

    def tick(self):
        if self.pose is None:
            self.publish_command()
            self.log_status('WAIT_ODOM')
            return
        if self.completed:
            self.publish_command()
            self.log_status('COMPLETE single lap')
            return
        name, tx, ty, stop = ROUTE[self.index]
        x, y, yaw = self.pose
        dist = math.hypot(tx - x, ty - y)
        # Avoid a last-centimetre heading flip toward the waypoint's lateral
        # offset: the camera must keep facing the gantry while stopped.
        tolerance = .16 if name in ('top_approach', 'top_stop',
                                     'lower_approach', 'lower_stop') else .10
        if dist < tolerance:
            if stop:
                self.publish_command()
                recent = (self.light_update is not None and
                          (self.get_clock().now() - self.light_update).nanoseconds < 700_000_000)
                if self.light_state == 'green' and recent:
                    self.green_streak += 1
                else:
                    self.green_streak = 0
                if self.green_streak < 5 and self.require_vision:
                    self.log_status(f'STOP {name}: light={self.light_state}')
                    return
                self.green_streak = 0
            self.log_status(f'REACHED {name} ({self.index + 1}/{len(ROUTE)})')
            self.index += 1
            if self.index == len(ROUTE):
                self.completed = True
            return
        if self.front_blocked():
            self.publish_command()
            self.log_status(f'BLOCKED before {name}')
            return
        error = wrap(math.atan2(ty - y, tx - x) - yaw)
        angular = self.heading_pid.calculate(error)
        if abs(error) > .35:
            linear = 0.0
        else:
            linear = max(.06, self.speed_pid.calculate(dist))
        self.publish_command(linear, angular)
        self.log_status(f'GO {name} ({self.index + 1}/{len(ROUTE)})')


def main():
    rclpy.init()
    node = Patrol()
    try:
        rclpy.spin(node)
    finally:
        node.publish_command()
        node.destroy_node()
        rclpy.shutdown()
