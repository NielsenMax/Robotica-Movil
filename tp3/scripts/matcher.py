#!/usr/bin/env python3

"""
ROS node for publishing calibration data for a camera
"""

# ROS 2 imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from message_filters import ApproximateTimeSynchronizer, Subscriber
from cv_bridge import CvBridge, CvBridgeError
import sensor_msgs_py.point_cloud2 as pc2
import std_msgs

# Standard library imports
import yaml
from typing import Optional

# Third-party imports
import cv2
#from cv2 import sfm
import numpy as np

class MatcherNode(Node):
    """ Node class """
    def __init__(self):
        super().__init__('matcher_node')

        self.matches_pub = self.create_publisher(Image, 'matches', 10)
        self.point_cloud_pub = self.create_publisher(PointCloud2, 'point_cloud', 10)

        self.left_image_sub = Subscriber(self, Image, '/left/image_rect_color')
        self.right_image_sub = Subscriber(self, Image, '/right/image_rect_color')

        self.sync = ApproximateTimeSynchronizer([self.left_image_sub, self.right_image_sub], queue_size=10, slop=0.1)


        self.sync.registerCallback(self.__cb)
        
        self.br = CvBridge()
        
        self.kpe : cv2.ORB = cv2.ORB_create()
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        self.load_calibration_data()

    def create_point_cloud_msg(points_3D: np.ndarray, frame_id: str = "map") -> PointCloud2:
        """
        Creates a PointCloud2 message from a NumPy array of 3D points.
        
        Args:
            points_3D: 3xN NumPy array of points in 3D (each column is [X, Y, Z]^T).
            frame_id: The reference frame to which the point cloud belongs (e.g., 'map', 'odom').
        
        Returns:
            PointCloud2: The ROS2 PointCloud2 message.
        """
        # Make sure points_3D is in (N, 3) shape, where N is the number of points
        if points_3D.shape[0] != 3:
            raise ValueError("Expected 3xN array for points_3D")
        
        # Convert points to a list of (x, y, z) tuples
        points = points_3D.T.tolist()

        # Define the fields for a PointCloud2 (x, y, z as float32)
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        # Create the PointCloud2 message
        header = rclpy.header.Header()
        header.frame_id = frame_id
        point_cloud_msg = pc2.create_cloud(header, fields, points)

        return point_cloud_msg
   
    def __process(self, left_msg: Image, right_msg: Image):
        # Convertimos los mensajes de imagen ROS a imágenes OpenCV
        left_frame : cv2.Mat  = self.br.imgmsg_to_cv2(left_msg)
        right_frame : cv2.Mat = self.br.imgmsg_to_cv2(right_msg)
        
        # Detectamos keypoints y sus descriptores en ambas imágenes
        left_kp : list[cv2.KeyPoint]
        left_desc: Optional[cv2.Mat]
        left_kp, left_desc = self.kpe.detectAndCompute(left_frame,None)
        
        right_kp: list[cv2.KeyPoint]
        right_desc: Optional[cv2.Mat]
        right_kp, right_desc = self.kpe.detectAndCompute(right_frame,None)
        
        if left_kp is None or right_kp is None:
            raise Exception('No keypoints found')
        
        # Buscamos los matches entre los descriptores de ambas imágenes mediante fuerza bruta
        matches = self.bf.match(left_desc, right_desc)
        # Filtramos los matches por distancia
        good_matches = [m for m in matches if m.distance < 30]

        # Dibuja los matches en una imagen
        match_img : cv2.Mat = cv2.drawMatches(left_frame, left_kp, right_frame, right_kp, good_matches, None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

        # Publicamos la imagen con los matches convertida a imagen ROS
        self.matches_pub.publish(self.br.cv2_to_imgmsg(match_img,encoding="bgr8"))
        
         # Convert keypoints to NumPy arrays
        left_pts = np.array([left_kp[m.queryIdx].pt for m in good_matches], dtype=np.float32).T  # Shape: (2, N)
        right_pts = np.array([right_kp[m.trainIdx].pt for m in good_matches], dtype=np.float32).T  # Shape: (2, N)

        # Ensure the points are in the shape (2, N)
        #left_pts = np.array(left_pts).T  # Shape: (2, N)
        #right_pts = np.array(right_pts).T  # Shape: (2, N)

        # Triangulate the 3D points in homogeneous coordinates
        triang_points_hom = cv2.triangulatePoints(self.left_proj, self.right_proj, left_pts, right_pts)

        # Convert homogeneous coordinates to Euclidean coordinates
        triang_points_euc = cv2.convertPointsFromHomogeneous(triang_points_hom.T).reshape(-1, 3)

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        
        header = std_msgs.msg.Header()
        header.stamp = self.get_clock().now().to_msg()  # Use the correct time function
        header.frame_id = "map"
        
        point_cloud_msg = pc2.create_cloud(header, fields, triang_points_euc)
        self.point_cloud_pub.publish(point_cloud_msg)
        self.get_logger().info('Publishing point cloud')

    def __cb(self, left_msg, right_msg):
        try:
            self.__process(left_msg, right_msg)
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
        except Exception as e:
            self.get_logger().error(f'Unhandled exception: {e}')

    def load_calibration_data(self):
        # Cargar los archivos YAML
        with open('calibrationdata/left.yaml', 'r') as file:
            left_data = yaml.safe_load(file)

        with open('calibrationdata/right.yaml', 'r') as file:
            right_data = yaml.safe_load(file)
        
        self.left_proj = np.array(left_data['projection_matrix']['data']).reshape((3, 4))
        self.right_proj = np.array(right_data['projection_matrix']['data']).reshape((3, 4))
        
def main(args=None):
    rclpy.init(args=args)

    node = MatcherNode()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
