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


class MatcherNode(Node):
    """ Node class """
    def __init__(self):
        super().__init__('matcher_node')

        self.pub = self.create_publisher(Image, 'matches', 10)

        self.left_image_sub = Subscriber(self, Image, '/left/image_rect_color')
        self.right_image_sub = Subscriber(self, Image, '/right/image_rect_color')

        self.sync = ApproximateTimeSynchronizer([self.left_image_sub, self.right_image_sub], queue_size=10, slop=0.1)


        self.sync.registerCallback(self.__cb)
        
        self.br = CvBridge()
        self.kpe = cv2.ORB_create()
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        
    
    def __cb(self, left_msg, right_msg):
        try:
            left_frame = self.br.imgmsg_to_cv2(left_msg)
            right_frame = self.br.imgmsg_to_cv2(right_msg)
            
            left_kp, left_desc = self.kpe.detectAndCompute(left_frame,None)
            right_kp, right_desc = self.kpe.detectAndCompute(right_frame,None)
            
            if left_desc is not None and right_desc is not None:
                matches = self.bf.match(left_desc, right_desc)
                good_matches = [m for m in matches if m.distance < 30]

                # Draw the matches
                match_img = cv2.drawMatches(left_frame, left_kp, right_frame, right_kp, good_matches, None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

                
                # Convert the image to ROS Image message and publish
                self.pub.publish(self.br.cv2_to_imgmsg(match_img,encoding="bgr8"))
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
        except Exception as e:
            self.get_logger().error(f'Unhandled exception: {e}')
        
def main(args=None):
    rclpy.init(args=args)

    node = MatcherNode()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
