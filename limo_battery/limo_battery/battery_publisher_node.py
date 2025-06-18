#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

class BatteryPublisherNode(Node):
    def __init__(self):
        super().__init__('battery_publisher')
        self.publisher = self.create_publisher(BatteryState, '/bat', 10)
        self.timer = self.create_timer(1.0, self.publish_battery)

    def publish_battery(self):
        msg = BatteryState()
        msg.percentage = 0.9
        self.get_logger().info(f'Publishing battery percentage: {msg.percentage}')
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = BatteryPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
