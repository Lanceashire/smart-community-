"""Move only signal lens visuals using Gazebo's UserCommands service.

The patrol never subscribes to this simulator state: it must use camera vision.
Durations are editable demo parameters; attached semifinal text mandates phases
but does not specify their durations.
"""
import json
import math
from pathlib import Path
import subprocess

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class TrafficSignals(Node):
    def __init__(self):
        super().__init__('community_signal_simulator')
        for name,value in [('red_seconds',10.0),('green_seconds',15.0),('yellow_seconds',3.0)]:
            self.declare_parameter(name,value)
        self.durations=[float(self.get_parameter(c+'_seconds').value) for c in ['red','green','yellow']]
        if min(self.durations)<=0:raise ValueError('Signal durations must be positive')
        share=Path(get_package_share_directory('community_demo'))
        self.lamps=json.loads((share/'worlds/layout.json').read_text())['lamps']
        self.state='red'
        self.debug=self.create_publisher(String,'/simulation/traffic_light_state',10)
        self.create_timer(.1,self.tick)
        self.get_logger().info('Camera-visible signals: red -> green -> yellow; simulation clock, no patrol state shortcut.')

    def tick(self):
        t=self.get_clock().now().nanoseconds/1e9 % sum(self.durations)
        wanted='red' if t<self.durations[0] else ('green' if t<sum(self.durations[:2]) else 'yellow')
        self.debug.publish(String(data=wanted))
        if wanted==self.state:return
        poses=[]
        for lamp in self.lamps:
            visible=(lamp['state']=='on')==(lamp['colour']==wanted)
            z=lamp['visible_z'] if visible else lamp['hidden_z']
            poses.append(f'pose {{ name: "{lamp["name"]}" position {{ x: {lamp["x"]} y: {lamp["y"]} z: {z} }} orientation {{ z: {math.sin(lamp["yaw"]/2)} w: {math.cos(lamp["yaw"]/2)} }} }}')
        try:
            result=subprocess.run(['gz','service','-s','/world/community/set_pose_vector',
                '--reqtype','gz.msgs.Pose_V','--reptype','gz.msgs.Boolean','--timeout','2000','--req',' '.join(poses)],
                capture_output=True,text=True,timeout=3)
            if result.returncode==0 and 'true' in result.stdout:
                self.state=wanted;self.get_logger().info(f'SIGNAL {wanted}')
            else:self.get_logger().error(f'Could not update lamp visuals: {result.stderr or result.stdout}')
        except subprocess.TimeoutExpired:self.get_logger().error('Gazebo lamp update timed out')


def main():
    rclpy.init();node=TrafficSignals()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:node.destroy_node();rclpy.try_shutdown()
