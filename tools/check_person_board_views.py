#!/usr/bin/env python3
"""Capture both faces of every card in a scene-only Gazebo inspection.

Robot pose resets are inspection operations, never autonomous-patrol evidence.
Requires demo.launch.py patrol:=false; refuses an active patrol restart service.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
import time

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    output = ROOT / args.output
    output.mkdir(parents=True, exist_ok=True)
    if (output / 'report.json').exists():
        raise RuntimeError('Use a new output directory to preserve earlier evidence.')
    layout = json.loads((ROOT / 'src/community_demo/worlds/layout.json').read_text())
    rclpy.init()
    node = rclpy.create_node('person_board_view_inspector')
    state = {'count': 0, 'frame': None, 'stamp_ns': 0, 'pose': None, 'pan': None}
    pan = node.create_publisher(Float64, '/model/patrol_bot/camera_pan', 10)

    def frame(message):
        if message.encoding not in ('rgb8', 'bgr8'):
            return
        pixels = np.frombuffer(message.data, np.uint8)
        if pixels.size < message.height * message.step:
            return
        pixels = pixels[:message.height * message.step].reshape(message.height, message.step)
        pixels = pixels[:, :message.width * 3].reshape(message.height, message.width, 3)
        state['frame'] = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR) if message.encoding == 'rgb8' else pixels.copy()
        state['stamp_ns'] = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        state['count'] += 1

    def pose(message):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        state['pose'] = [p.x, p.y, p.z, yaw]

    def joint(message):
        if 'camera_pan_joint' in message.name:
            state['pan'] = message.position[message.name.index('camera_pan_joint')]

    node.create_subscription(Image, '/camera/image', frame, qos_profile_sensor_data)
    node.create_subscription(Odometry, '/model/patrol_bot/ground_truth', pose, qos_profile_sensor_data)
    node.create_subscription(JointState, '/world/community/model/patrol_bot/joint_state', joint, qos_profile_sensor_data)

    def spin(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)

    def move(x, y, yaw):
        if any(name == '/patrol/restart' for name, _ in node.get_service_names_and_types()):
            raise RuntimeError('An autonomous patrol is present; stop it before inspection.')
        request = f'name: "patrol_bot" position {{ x: {x} y: {y} z: .002 }} orientation {{ z: {math.sin(yaw/2)} w: {math.cos(yaw/2)} }}'
        result = subprocess.run(['gz', 'service', '-s', '/world/community/set_pose', '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean', '--timeout', '2000', '--req', request], capture_output=True, text=True, timeout=4)
        if result.returncode or 'true' not in result.stdout:
            raise RuntimeError(result.stderr + result.stdout)
        before = state['count']
        for _ in range(5):
            pan.publish(Float64(data=0.0))
            spin(.25)
        actual = state['pose']
        if state['count'] - before < 5 or state['frame'] is None or actual is None:
            raise RuntimeError('Fresh camera and actual robot pose evidence unavailable.')
        yaw_error = abs(math.atan2(math.sin(actual[3] - yaw), math.cos(actual[3] - yaw)))
        if math.hypot(actual[0] - x, actual[1] - y) > .03 or yaw_error > .03:
            raise RuntimeError('Robot failed to settle at the requested inspection viewpoint.')
        if state['pan'] is None or abs(state['pan']) > .03:
            raise RuntimeError('Actual camera encoder is not facing forward.')

    rows = []
    error = None
    spin(2)
    if any(name == '/patrol/restart' for name, _ in node.get_service_names_and_types()):
        node.destroy_node()
        rclpy.shutdown()
        raise RuntimeError('Run the world with patrol:=false.')
    try:
        for person in layout['people']:
            for side, sign in [('front', 1), ('back', -1)]:
                yaw = person['yaw']
                x = person['x'] + sign * .55 * math.cos(yaw)
                y = person['y'] + sign * .55 * math.sin(yaw)
                heading = yaw + (math.pi if sign == 1 else 0)
                move(x, y, heading)
                filename = f"{person['name']}-{person['asset']}-{side}.png"
                if not cv2.imwrite(str(output / filename), state['frame']):
                    raise RuntimeError('Could not save the inspection frame.')
                row = {'person': person['name'], 'asset': person['asset'], 'side': side,
                       'printed_face_yaw': yaw, 'camera_request': [x, y, heading],
                       'actual_robot_pose': state['pose'], 'actual_camera_pan': state['pan'],
                       'image_source_stamp_ns': state['stamp_ns'], 'image': filename,
                       'sha256': hashlib.sha256((output / filename).read_bytes()).hexdigest()}
                rows.append(row)
                print('CAPTURED', person['name'], person['asset'], side, flush=True)
    except Exception as exc:
        error = str(exc)
        raise
    finally:
        try:
            move(*layout['spawn'])
        finally:
            report = {'captures_completed': len(rows) == 2 * len(layout['people']) and error is None,
                      'scope': 'Scene-only pose resets. Front/back rendering inspection, not navigation or recognition accuracy.',
                      'error': error, 'views': rows}
            (output / 'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
