"""Adapt valid no-return beams for Karto without inventing obstacle endpoints.

Karto ignores >= range_max and +inf. A 5.99 m beam, with mapping threshold
5.95 m and sensor maximum 6 m, traces free space without marking its endpoint.
Collision control continues consuming the unmodified /scan topic.
"""
import copy,math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from community_demo.sensor_validation import valid_laser_scan,stamp_ns

def mapping_scan(message):
    if not valid_laser_scan(message) or message.range_max<6:return None
    result=copy.deepcopy(message)
    result.ranges=[5.99 if r==math.inf else r for r in message.ranges]
    return result

class Adapter(Node):
    def __init__(self):
        super().__init__('slam_scan_adapter');self.previous=None
        self.pub=self.create_publisher(LaserScan,'/scan_slam',qos_profile_sensor_data)
        self.create_subscription(LaserScan,'/scan',self.scan,qos_profile_sensor_data)
    def scan(self,message):
        source=stamp_ns(message.header.stamp)
        if source is None or not 0<=self.get_clock().now().nanoseconds-source<=500_000_000:return
        if self.previous is not None and source<=self.previous:return
        result=mapping_scan(message)
        if result is None:return
        self.previous=source;self.pub.publish(result)

def main():
    rclpy.init();n=Adapter()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.destroy_node();rclpy.try_shutdown()
