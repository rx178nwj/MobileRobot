#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class DummyCam(Node):
    def __init__(self):
        super().__init__('dummy_cam')
        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.create_timer(0.2, self.cb)
        self.get_logger().info('Dummy camera publishing /camera/image_raw at 5Hz')

    def cb(self):
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = 240
        msg.width = 320
        msg.encoding = 'bgr8'
        msg.step = 320 * 3
        msg.data = bytes(240 * 320 * 3)  # black image
        self.pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(DummyCam())

if __name__ == '__main__':
    main()
