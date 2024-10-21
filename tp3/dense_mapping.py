#!/usr/bin/env python3

"""
ROS node for publishing calibration data for a camera
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from message_filters import ApproximateTimeSynchronizer, Subscriber
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from stereo_msgs.msg import DisparityImage
import std_msgs
import sensor_msgs_py.point_cloud2 as pc2

from utils import read_calibation_data

class DenseMapperNode(Node):
    """ Node class """
    def __init__(self):
        super().__init__('dense_mapper_node')
        
        left_data = read_calibation_data("./calibrationdata/left.yaml")
        right_data = read_calibation_data("./calibrationdata/right.yaml")

        self.pub = self.create_publisher(PointCloud2, 'point_cloud', 10)

        self.sub = Subscriber(self, DisparityImage, '/disparity_new')

        self.sync = ApproximateTimeSynchronizer([self.sub], queue_size=10, slop=0.1)


        self.sync.registerCallback(self.__cb)
        
        self.br = CvBridge()
        
        left_cam_to_body = np.array([[0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975],
                                    [0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768],
                                    [-0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949],
                                    [0.0, 0.0, 0.0, 1.0]])
        body_to_right_cam = np.linalg.inv(np.array([[0.0125552670891, -0.999755099723, 0.0182237714554, -0.0198435579556],
                                    [0.999598781151, 0.0130119051815, 0.0251588363115, 0.0453689425024],
                                    [-0.0253898008918, 0.0179005838253, 0.999517347078, 0.00786212447038],
                                    [0.0, 0.0, 0.0, 1.0]]))
        
        W = np.dot(body_to_right_cam, left_cam_to_body)
        
        # Extract rotation matrix (3x3)
        R = W[0:3, 0:3]

        # Extract translation vector (3x1)
        T = W[0:3, 3]
        _, _, _, _, Q, _, _ = cv2.stereoRectify(
            cameraMatrix1=np.array(left_data['camera_matrix']).reshape(3,3),
            distCoeffs1=np.array(left_data['distortion_coefficients']),
            cameraMatrix2=np.array(right_data['camera_matrix']).reshape(3,3),
            distCoeffs2=np.array(right_data['distortion_coefficients']),
            imageSize=(left_data['image_width'], left_data['image_height']),
            R1=np.array(left_data['rectification_matrix']).reshape(3, 3),
            R2=np.array(right_data['rectification_matrix']).reshape(3, 3),
            flags=cv2.CALIB_ZERO_DISPARITY,
            R=R,
            T=T,
            alpha=-1
        )
        self.Q = Q
        
    
    def __cb(self, disparity):
        try:
            disparity_image = disparity.image
            disparity_array = self.br.imgmsg_to_cv2(disparity_image, desired_encoding='32FC1')

            # Reproject the image to 3D
            points = cv2.reprojectImageTo3D(disparity_array, self.Q)
            points = points.reshape(-1, 3)
            
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
            ]
            
            header = std_msgs.msg.Header()
            header.stamp = self.get_clock().now().to_msg()  # Use the correct time function
            header.frame_id = "map"
            
            point_cloud_msg = pc2.create_cloud(header, fields, points)
            
            self.pub.publish(point_cloud_msg)
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
        except Exception as e:
            self.get_logger().error(f'Unhandled exception: {e}')
        
def main(args=None):
    rclpy.init(args=args)

    node = DenseMapperNode()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
