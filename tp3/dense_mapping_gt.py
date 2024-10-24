#!/usr/bin/env python3

"""
ROS node for publishing calibration data for a camera
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField, Imu
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
import transforms3d.quaternions as tq



class DenseMapperGTNode(Node):
    """ Node class """
    def __init__(self):
        super().__init__('dense_mapper_gt_node')

        self.pub = self.create_publisher(PointCloud2, 'point_cloud_gt', 10)


        self.points_sub = Subscriber(self, PointCloud2, '/point_cloud')
        self.gt_sub = Subscriber(self, Imu, '/imu0')

        self.sync = ApproximateTimeSynchronizer([self.gt_sub, self.points_sub], queue_size=10, slop=0.1)

        self.sync.registerCallback(self.__cb)
    
    def __cb(self, gt: Imu, points: PointCloud2):
        quat = [
            gt.orientation.x,
            gt.orientation.y,
            gt.orientation.z,
            gt.orientation.w
        ]
        print(quat)
        
        # Convert quaternion to rotation matrix
        rotation_matrix = tq.quat2mat(quat)

        # Apply transformation
        transformed_points = self.transform_point_cloud(points, rotation_matrix)

        # Create new PointCloud2 message
        new_point_cloud = pc2.create_cloud(points.header, points.fields, transformed_points)
        
        print(new_point_cloud)
        # Publish the new point cloud
        self.pub.publish(new_point_cloud)

    def transform_point_cloud(self, point_cloud, rotation_matrix):
        # Read points from point cloud
        points_list = list(pc2.read_points(point_cloud, skip_nans=True))
        
        # Transform each point
        transformed_points = []
        for point in points_list:
            pt = np.array([point[0], point[1], point[2], 1.0])
            transformed_pt = np.dot(rotation_matrix, pt)
            transformed_points.append((transformed_pt[0], transformed_pt[1], transformed_pt[2]))

        return transformed_points
        
def main(args=None):
    rclpy.init(args=args)

    node = DenseMapperGTNode()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
