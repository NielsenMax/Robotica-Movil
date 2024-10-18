#!/usr/bin/env python3

"""
ROS node for publishing calibration data for a camera
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from message_filters import ApproximateTimeSynchronizer, Subscriber
from cv_bridge import CvBridge, CvBridgeError
import cv2


class ExtractorNode(Node):
    """ Node class """
    def __init__(self):
        super().__init__('extractor_node')

        self.left_pub = self.create_publisher(Image, 'left/features', 10)
        self.right_pub = self.create_publisher(Image, 'right/features', 10)

        self.left_image_sub = Subscriber(self, Image, '/left/image_rect_color')
        self.right_image_sub = Subscriber(self, Image, '/right/image_rect_color')

        self.left_sync = ApproximateTimeSynchronizer([self.left_image_sub], queue_size=10, slop=0.1)
        self.right_sync = ApproximateTimeSynchronizer([self.right_image_sub], queue_size=10, slop=0.1)

        self.left_sync.registerCallback(self.callback_left)
        self.right_sync.registerCallback(self.callback_right)
        
        self.br = CvBridge()
        self.kpe = cv2.ORB_create()
        
    def callback_right(self, msg):
        self.__cb(self.right_pub, msg)

    def callback_left(self, msg):
        self.__cb(self.left_pub, msg)
    
    def __cb(self, pub, msg):
        try:
            frame = self.br.imgmsg_to_cv2(msg)
            keypoints, descriptors = self.kpe.detectAndCompute(frame,None)
            img = cv2.drawKeypoints(frame, keypoints, None)
            pub.publish(self.br.cv2_to_imgmsg(img,encoding="bgr8"))
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
        except Exception as e:
            self.get_logger().error(f'Unhandled exception: {e}')
        
def main(args=None):
    rclpy.init(args=args)

    node = ExtractorNode()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
