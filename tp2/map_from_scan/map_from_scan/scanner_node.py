#!/usr/bin/env python3

"""
ROS node for 2D lidar scan to map
"""

import sys
import rclpy
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
import math
from rclpy.node import Node


class Processor:       
    def process(self, args, ranges):
        self.x = args['x']
        self.y = args['y']
        self.orientation = args['orientation']
        self.min_range = args['range_min']
        self.max_range = args['range_max']
        self.angle_min = args['angle_min']
        self.angle_increment = args['angle_increment']
        clean_ranges = self.__clean(ranges)
        return self.__split_by_cylinder(clean_ranges)
        
    def __clean(self, ranges):
        clean_data = []
        for i in range(0, len(ranges)):
            length = ranges[i]
            if length > self.max_range or length < self.min_range:
                length = None
                
            angle = self.orientation + self.angle_min + self.angle_increment * (i+1)
            clean_data.append({
                'angle': angle,
                'range': length
            })
        return clean_data
    
    def __polar_to_xy(self, polar):
        x = self.x + polar['range'] * math.cos(polar['angle'])
        y = self.y + polar['range'] * math.sin(polar['angle'])
        return {'x': x, 'y': y}
    
    def __middle(self, a, b):
        return (b + a)/2
    
    def __calc_cylinder(self, a, b):
        radius = math.hypot(a['x'] - b['x'], a['y'] - b['y'])/2
        return {
            'x': self.__middle(a['x'], b['x']),
            'y': self.__middle(a['y'], b['y']),
            'radius':  radius
        }
    
    def __split_by_cylinder(self, ranges):
        # This loop is to loop the start of the array because 
        # it may start at the middle of a cylinder
        loop = ranges
        for range in ranges:
            if range:
                loop = loop[1:] + [loop[0]]
            else:
                break
            
        cylinders = []
        rigth_side = None
        left_side = None
        for range in loop:
            if range['range']:
                if not rigth_side:
                    rigth_side = self.__polar_to_xy(range)
                else: 
                    left_side = self.__polar_to_xy(range)
                continue
            
            if left_side:
                cylinders.append(self.__calc_cylinder(rigth_side, left_side))
                rigth_side = None
                left_side = None
        
        # This is because it may be a last cylinder
        if left_side:
            cylinders.append(self.__calc_cylinder(rigth_side, left_side))
        return cylinders

class ScannerNode(Node):
    """ Node class """
    def __init__(self):
        super().__init__('scanner_node')
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)

    def __place_cylinder(self, x, y, radius, id):
        marker = Marker()
        marker.header.frame_id = "base_scan"
        marker.ns = "laser_points"
        marker.id = id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 1.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 2*radius
        marker.scale.y = 2*radius
        marker.scale.z = 1.0
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0

        # Give some lifetime to the marker so there's a chance
        # to see them all in the same picture
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = int(0.2 * 1e9)
        self.marker_pub.publish(marker)

    def scan_cb(self, msg):
        """ Scan subscriber callback """
        ranges = msg.ranges
        args = {
            'x': 0.0,
            'y': 0.0,
            'orientation': 0.0,
            'range_min': msg.range_min,
            'range_max': msg.range_max,
            'angle_min': msg.angle_min,
            'angle_increment': msg.angle_increment
        }
        cylinders = Processor().process(args, ranges)
        
        for i, cylinder in enumerate(cylinders):
            self.__place_cylinder(cylinder['x'], cylinder['y'], cylinder['radius'], i)
        
        
def main(args=None):
    rclpy.init(args=args)

    node = ScannerNode()

    rclpy.spin(node)

    rclpy.shutdown()

    node.get_clock().now()

if __name__ == '__main__':
    main()