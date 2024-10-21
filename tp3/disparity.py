#!/usr/bin/env python3

"""
ROS node for publishing calibration data for a camera
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from message_filters import ApproximateTimeSynchronizer, Subscriber
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from stereo_msgs.msg import DisparityImage

class DisparityNode(Node):
    """ Node class """
    def __init__(self):
        super().__init__('disparity_node')

        self.pub = self.create_publisher(DisparityImage, 'disparity_new', 10)

        self.left_image_sub = Subscriber(self, Image, '/left/image_rect_color')
        self.right_image_sub = Subscriber(self, Image, '/right/image_rect_color')

        self.sync = ApproximateTimeSynchronizer([self.left_image_sub, self.right_image_sub], queue_size=10, slop=0.1)


        self.sync.registerCallback(self.__cb)
        
        self.br = CvBridge()
        self.stereo = cv2.StereoBM_create()
        
    
    def __cb(self, left_msg, right_msg):
        try:
            left_frame = self.br.imgmsg_to_cv2(left_msg)
            right_frame = self.br.imgmsg_to_cv2(right_msg)
          
            disparity = self.stereo.compute(left_frame, right_frame)  
          
            disparity = disparity.astype(np.float32) / 16.0
          
            disparity_image_msg = DisparityImage()
            disparity_image_msg.header.stamp = self.get_clock().now().to_msg()
            disparity_image_msg.header.frame_id = left_msg.header.frame_id
            
            disparity_image_msg.image = self.br.cv2_to_imgmsg(disparity, encoding="32FC1")
            disparity_image_msg.min_disparity = float(np.min(disparity))
            disparity_image_msg.max_disparity = float(np.max(disparity))
            disparity_image_msg.f = 496.2620544433594  # Focal length, replace with actual value
            disparity_image_msg.t = 0.11547816544771194  # Baseline, replace with actual value
            disparity_image_msg.delta_d = 1.0 / 16.0  # Disparity increment
            
            self.pub.publish(disparity_image_msg)
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
        except Exception as e:
            self.get_logger().error(f'Unhandled exception: {e}')
        
def main(args=None):
    rclpy.init(args=args)

    node = DisparityNode()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
