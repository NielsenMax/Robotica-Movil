#!/usr/bin/env python3

"""
ROS node for publishing calibration data for a camera
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from message_filters import Subscriber
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
import transforms3d.quaternions as tq
import csv
from utils import matrix_from_pose


class DenseMapperGTNode(Node):
    """ Node class """
    def __init__(self):
        super().__init__('dense_mapper_gt_node')

        self.pub = self.create_publisher(PointCloud2, 'point_cloud_gt', 10)


        self.points_sub = Subscriber(self, PointCloud2, '/point_cloud')

        self.data = self.parse_csv("MH_01_easy/mav0/state_groundtruth_estimate0/data.csv")

        self.points_sub.registerCallback(self.__cb)
    
    def __cb(self, points: PointCloud2):
        rotation_matrix = self.find_matrix(points.header.stamp)
        # Apply transformation
        transformed_points = self.transform_point_cloud(points, rotation_matrix)

        # Create new PointCloud2 message
        new_point_cloud = pc2.create_cloud(points.header, points.fields, transformed_points)
        
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

    def parse_csv(self, file_path):
        data = []
        
        with open(file_path, mode='r') as csv_file:
            csv_reader = csv.DictReader(csv_file)
            for row in csv_reader:
                entry = {
                    "timestamp": int(row["#timestamp"]),
                    "x": float(row[" p_RS_R_x [m]"]),
                    "y": float(row[" p_RS_R_y [m]"]),
                    "z": float(row[" p_RS_R_z [m]"]),
                    "qw": float(row[" q_RS_w []"]),
                    "qx": float(row[" q_RS_x []"]),
                    "qy": float(row[" q_RS_y []"]),
                    "qz": float(row[" q_RS_z []"])
                }
                data.append(entry)

        return data
    
    def find_matrix(self, stamp):
        print(stamp)
        msg_time = int(stamp.sec * 1e9 + stamp.nanosec) - 327733401425386240 - 214296034816 # Convert message time to nanoseconds
        print(f"Message time: {msg_time}")

        closest_entry = min(self.data, key=lambda x: abs(x["timestamp"] - msg_time))

        print(f"Closest entry timestamp: {closest_entry['timestamp']}")
        print(f"Difference: {abs(closest_entry['timestamp'] - msg_time)}")
        print("----")

        translation = np.array([
            closest_entry["x"],
            closest_entry["y"],
            closest_entry["z"],])
        quaternion = np.array([
            closest_entry["qw"],
            closest_entry["qx"],
            closest_entry["qy"],
            closest_entry["qz"],
        ])

        return matrix_from_pose(translation, quaternion)
        
def main(args=None):
    rclpy.init(args=args)

    node = DenseMapperGTNode()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
