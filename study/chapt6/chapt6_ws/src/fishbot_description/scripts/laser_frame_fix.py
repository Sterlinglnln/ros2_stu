#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LaserFrameFixer(Node):
    def __init__(self):
        super().__init__('laser_frame_fixer')
        self.sub = self.create_subscription(
            LaserScan,
            '/scan_raw',
            self.scan_callback,
            10
        )
        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        self.get_logger().info("Laser frame fixer node started. Listening on /scan_raw")

    def scan_callback(self, msg):
        msg.header.frame_id = 'laser_link'
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LaserFrameFixer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
