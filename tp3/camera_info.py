#!/usr/bin/env python3

"""
ROS node for publishing calibration data for a camera
"""

import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from message_filters import ApproximateTimeSynchronizer, Subscriber


class CameraInfoNode(Node):
    """ Node class """
    def __init__(self):
        super().__init__('camera_info_node')

        self.left_pub = self.create_publisher(CameraInfo, 'left/camera_info', 10)
        self.right_pub = self.create_publisher(CameraInfo, 'right/camera_info', 10)
        
        self.left_data = self.__read_data("./calibrationdata/left.yaml")
        self.right_data = self.__read_data("./calibrationdata/right.yaml")

        self.left_image_sub = Subscriber(self, Image, '/left/image_raw')
        self.right_image_sub = Subscriber(self, Image, '/right/image_raw')

        self.left_sync = ApproximateTimeSynchronizer([self.left_image_sub], queue_size=10, slop=0.1)
        self.right_sync = ApproximateTimeSynchronizer([self.right_image_sub], queue_size=10, slop=0.1)

        self.left_sync.registerCallback(self.callback_left)
        self.right_sync.registerCallback(self.callback_right)
        
    def callback_right(self, msg):
        self.__publish(self.right_pub, self.right_data, msg.header)

    def callback_left(self, msg):
        self.__publish(self.left_pub, self.left_data, msg.header)
           
    def __read_data(self, path):
        # Load the YAML file
        with open(path, 'r') as file:
            camera_info = yaml.safe_load(file)

        # Extract the information
        image_width = camera_info['image_width']
        image_height = camera_info['image_height']
        camera_name = camera_info['camera_name']

        camera_matrix = camera_info['camera_matrix']['data']
        distortion_model = camera_info['distortion_model']
        distortion_coefficients = camera_info['distortion_coefficients']['data']
        rectification_matrix = camera_info['rectification_matrix']['data']
        projection_matrix = camera_info['projection_matrix']['data']

        return {
            'image_width': image_width,
            'image_height': image_height,
            'camera_name': camera_name,
            'camera_matrix': camera_matrix,
            'distortion_model': distortion_model,
            'distortion_coefficients': distortion_coefficients,
            'rectification_matrix': rectification_matrix,
            'projection_matrix': projection_matrix,
        }

    def __publish(self, pub, data, header):
        msg = self.__create_camera_info_msg(data)
        msg.header = header
        pub.publish(msg)

    def __create_camera_info_msg(self, data):
        msg = CameraInfo()
        msg.width = data['image_width']
        msg.height = data['image_height']
        msg.distortion_model = data['distortion_model']
        msg.d = data['distortion_coefficients']
        msg.k = data['camera_matrix']
        msg.r = data['rectification_matrix']
        msg.p = data['projection_matrix']
        return msg


def main(args=None):
    rclpy.init(args=args)

    node = CameraInfoNode()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
