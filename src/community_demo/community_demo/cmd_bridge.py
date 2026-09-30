"""Adapt the RM TwistStamped safety output to Gazebo DiffDrive's Twist input."""

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


class CommandBridge(Node):
    def __init__(self):
        super().__init__('community_command_bridge')
        self.publisher = self.create_publisher(Twist, '/model/patrol_bot/cmd_vel', 10)
        self.create_subscription(TwistStamped, '/cmd_vel', self.forward, 10)

    def forward(self, message):
        self.publisher.publish(message.twist)


def main():
    rclpy.init()
    node = CommandBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
