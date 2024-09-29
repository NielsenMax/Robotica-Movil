#!/usr/bin/env python3

"""
ROS node for 2D odometry dump
"""

import sys
import rclpy
from sensor_msgs.msg import LaserScan

class DumpScan:
    """ Node class """
    def __init__(self):

        # Node subscribers
        node.create_subscription(LaserScan, 'scan', self.scan_cb, 10)

    def scan_cb(self, msg):
        """ Scan subscriber callback """
        ranges = msg.ranges
        print(",".join(map(str, ranges)))

if __name__ == '__main__':
    rclpy.init(args=sys.argv)
    node = rclpy.create_node('dump_scan')

    DumpScan()
    rclpy.spin(node)
    rclpy.shutdown()
    
    my_node.get_clock().now()