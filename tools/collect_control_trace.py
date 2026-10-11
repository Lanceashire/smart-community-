#!/usr/bin/env python3
"""Read-only planner and smoothed command recorder; no control publishers."""
import argparse,json,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);a=p.parse_args()
    rclpy.init();node=Node('control_trace_recorder');rows={'planner':[],'navigation':[],'manager':[]}
    def text(m):
        item=json.loads(m.data);item['received_monotonic']=time.monotonic();rows['planner'].append(item)
    def command(kind,m):
        rows[kind].append({'stamp':m.header.stamp.sec+m.header.stamp.nanosec*1e-9,
                           'linear':m.twist.linear.x,'angular':m.twist.angular.z})
    node.create_subscription(String,'/patrol/control_trace',text,100)
    node.create_subscription(TwistStamped,'/cmd_vel_nav',lambda m:command('navigation',m),100)
    node.create_subscription(TwistStamped,'/cmd_vel',lambda m:command('manager',m),100)
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        out=Path(a.output);out.parent.mkdir(parents=True,exist_ok=True)
        out.write_text(json.dumps(rows,separators=(',',':'))+'\n')
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
