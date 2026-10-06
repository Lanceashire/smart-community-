"""Read-only audit of actual signal lenses; Gazebo plugin performs switching."""
import json
from collections import deque
import threading
import time
from pathlib import Path
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.stringmsg_pb2 import StringMsg


class TrafficSignals(Node):
    def __init__(self):
        super().__init__('community_signal_simulator')
        share=Path(get_package_share_directory('community_demo'))
        self.lamps=json.loads((share/'worlds/layout.json').read_text())['lamps']
        self.debug=self.create_publisher(String,'/simulation/traffic_light_state',10)
        self.stamped=self.create_publisher(String,'/simulation/traffic_light_stamped',10)
        self.phase_history=deque(maxlen=1024)
        self.diagnostics=self.create_publisher(String,'/simulation/traffic_light_diagnostics',10)
        self.lock=threading.Lock()
        self.positions={};self.pose_received=0.;self.engine={};self.state='unknown';self.transitions=0
        self.gazebo=GzNode()
        self.gazebo.subscribe(Pose_V,'/world/community/pose/info',self.on_poses)
        self.gazebo.subscribe(StringMsg,'/simulation/traffic_light_engine',self.on_engine)
        self.create_timer(.1,self.tick)

    def on_poses(self,message):
        positions={p.name:p.position.z for p in message.pose if p.name.startswith('light_')}
        if not all(l['name'] in positions for l in self.lamps):return
        with self.lock:
            self.positions=positions;self.pose_received=time.monotonic()
            phase='unknown'
            for colour in ('red','green','yellow'):
                if all(abs(positions[l['name']]-(l['visible_z'] if
                       ((l['state']=='on')==(l['colour']==colour)) else l['hidden_z']))<.002
                       for l in self.lamps):phase=colour;break
            t=message.header.stamp.sec*1_000_000_000+message.header.stamp.nsec
            if not self.phase_history or (t>self.phase_history[-1][0] and (t-self.phase_history[-1][0]>=50_000_000 or phase!=self.phase_history[-1][1])):self.phase_history.append((t,phase))

    def on_engine(self,message):
        try:self.engine=json.loads(message.data)
        except ValueError:pass

    def actual_phase(self):
        with self.lock:positions,received=self.positions.copy(),self.pose_received
        if time.monotonic()-received>.5:return 'unknown'
        for colour in ('red','green','yellow'):
            if all(abs(positions[l['name']]-(l['visible_z'] if
                       ((l['state']=='on')==(l['colour']==colour)) else l['hidden_z']))<.002
                   for l in self.lamps):return colour
        return 'unknown'

    def tick(self):
        measured=self.actual_phase()
        if measured!=self.state:
            self.state=measured;self.transitions+=1
            self.get_logger().info('SIGNAL '+measured+' independently observed from twelve lens poses')
        self.debug.publish(String(data=measured))
        with self.lock:history=list(self.phase_history)
        self.stamped.publish(String(data=json.dumps({'source':'Actual twelve Gazebo lens poses, evaluator only','samples':history})))
        self.diagnostics.publish(String(data=json.dumps({
            'measured':measured,'requests':0,'acknowledged':0,'timeouts':0,'failures':0,
            'transitions':self.transitions,'pose_age_wall_s':round(time.monotonic()-self.pose_received,3),
            'phase_source':'Actual Gazebo lens poses, read only',
            'native_engine':self.engine,'service_path':'Removed; switching inside simulation update'})))


def main():
    rclpy.init();node=TrafficSignals()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:node.destroy_node();rclpy.try_shutdown()
