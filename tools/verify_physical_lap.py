#!/usr/bin/env python3
"""Verify a lap using Gazebo physical pose, not wheel odometry or log text alone."""

import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', default='log/physical_lap_evidence.json')
    parser.add_argument('--timeout', type=float, default=260)
    parser.add_argument('--restart', action='store_true', help='Restart a completed lap without closing Gazebo.')
    args = parser.parse_args()
    from community_demo.patrol import ROUTE
    expected_waypoints=len(ROUTE)
    rclpy.init()
    node = rclpy.create_node('physical_lap_verifier')
    samples, reached = [], []
    state = {'status': '', 'done': False, 'fault': False, 'started': False, 'pose': None}
    car_detections = {}
    layout=json.loads((Path(__file__).resolve().parents[1]/'src/community_demo/worlds/layout.json').read_text())
    cars=layout['cars']

    def pose(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        roll = math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y))
        pitch = math.asin(max(-1., min(1., 2*(q.w*q.y-q.z*q.x))))
        samples.append([msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9,
                        p.x, p.y, p.z, max(abs(roll), abs(pitch))])
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        state['pose'] = (p.x, p.y, yaw)

    def scan(msg):
        if state['pose'] is None or not msg.ranges or msg.angle_increment <= 0:
            return
        x, y, yaw = state['pose']
        if not 1.0 < x < 1.2:
            return
        for i, car in enumerate(cars, 1):
            car_y=car['y']
            if abs(y-car_y) > .1:
                continue
            bearing = math.atan2(car_y-y, car['x']-x)
            local_angle = math.atan2(math.sin(bearing-yaw), math.cos(bearing-yaw))
            index = round((local_angle-msg.angle_min)/msg.angle_increment)
            if not 0 <= index < len(msg.ranges):
                continue
            value = msg.ranges[index]
            expected = (car['x']-car['size_m'][0]/2-x)/math.cos(bearing)
            if math.isfinite(value) and abs(value-expected) < .06:
                car_detections[f'car_{i}'] = {'range_m': value, 'expected_face_range_m': expected,
                                              'robot_xy': [x, y]}

    def status(msg):
        state['status'] = msg.data
        if msg.data.startswith('REACHED ') and (not reached or reached[-1] != msg.data):
            reached.append(msg.data)
            print(msg.data, flush=True)
        if msg.data.startswith(('GO ', 'REACHED ')):
            state['started'] = True
        state['done'] = state['started'] and msg.data.startswith('COMPLETE single lap')
        state['fault'] = msg.data.startswith('FAULT ')

    node.create_subscription(Odometry, '/model/patrol_bot/ground_truth', pose, 100)
    node.create_subscription(String, '/patrol/status', status, 20)
    node.create_subscription(LaserScan, '/scan', scan, qos_profile_sensor_data)
    if args.restart:
        client = node.create_client(Trigger, '/patrol/restart')
        if not client.wait_for_service(timeout_sec=5):
            raise RuntimeError('Patrol restart service is unavailable.')
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        if not future.done() or not future.result().success:
            raise RuntimeError('Patrol is not at a completed start area; cannot restart.')
        print(future.result().message, flush=True)
    deadline = time.monotonic() + args.timeout
    while time.monotonic() < deadline and not state['done'] and not state['fault']:
        rclpy.spin_once(node, timeout_sec=.2)
    distance = sum(math.hypot(b[1]-a[1], b[2]-a[2]) for a, b in zip(samples, samples[1:]))
    span_x = max((s[1] for s in samples), default=0) - min((s[1] for s in samples), default=0)
    span_y = max((s[2] for s in samples), default=0) - min((s[2] for s in samples), default=0)
    max_tilt = max((s[4] for s in samples), default=math.inf)
    passed = state['done'] and len(reached) == expected_waypoints and distance > 8 and span_x > 2.5 and span_y > 2.5 and max_tilt < .35 and len(car_detections) == 3
    report = dict(passed=passed, status=state['status'], physical_distance_m=distance,
                  span_x_m=span_x, span_y_m=span_y,
                  max_tilt_degrees=math.degrees(max_tilt) if math.isfinite(max_tilt) else None,
                  parked_car_lidar_detections=car_detections,
                  reached=reached, first_pose=samples[0] if samples else None,
                  last_pose=samples[-1] if samples else None, samples=samples)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2))
    print(json.dumps({k: v for k, v in report.items() if k != 'samples'}, indent=2), flush=True)
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if passed else 1)


if __name__ == '__main__':
    main()
