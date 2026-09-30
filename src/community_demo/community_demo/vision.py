"""Camera-only traffic colour baseline for the generated proof-of-run world."""

from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Image
from std_msgs.msg import String


class TrafficVision(Node):
    def __init__(self):
        super().__init__('community_traffic_vision')
        self.publisher = self.create_publisher(String, '/perception/traffic_light', 10)
        self.create_subscription(Image, '/camera/image', self.on_image, 10)
        self.history = deque(maxlen=3)
        self.last_state = None

    def on_image(self, msg):
        if msg.encoding not in ('rgb8', 'bgr8') or msg.step < msg.width * 3:
            return
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        if raw.size < msg.height * msg.step:
            return
        frame = raw[:msg.height * msg.step].reshape(msg.height, msg.step)[:, :msg.width * 3]
        frame = frame.reshape(msg.height, msg.width, 3)
        if msg.encoding == 'rgb8':
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # The gantry is above camera height; ignore brightly coloured people
        # and the ground in the lower half of the frame.
        roi = hsv[:int(msg.height * .58), :, :]
        red = cv2.inRange(roi, (0, 130, 95), (12, 255, 255)) | cv2.inRange(
            roi, (170, 130, 95), (179, 255, 255))
        yellow = cv2.inRange(roi, (16, 115, 105), (38, 255, 255))
        green = cv2.inRange(roi, (40, 100, 85), (85, 255, 255))
        counts = {'red': cv2.countNonZero(red), 'yellow': cv2.countNonZero(yellow),
                  'green': cv2.countNonZero(green)}
        best = max(counts, key=counts.get)
        state = best if counts[best] >= 30 else 'unknown'
        self.history.append(state)
        stable = state if len(self.history) == 3 and all(x == state for x in self.history) else 'unknown'
        result = String()
        result.data = stable
        self.publisher.publish(result)
        if stable != self.last_state:
            self.get_logger().info(f'camera traffic light: {stable}; pixels={counts}')
            self.last_state = stable


def main():
    rclpy.init()
    node = TrafficVision()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
