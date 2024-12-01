#!/usr/bin/env python3

import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from message_filters import ApproximateTimeSynchronizer, Subscriber
import subprocess
from threading import Thread, Event


class BagStereoNode(Node):
    """Node to play a bag, launch stereo processing, and publish CameraInfo."""

    def __init__(self):
        super().__init__('bag_stereo_node')

        # Load calibration data
        self.left_calibration = self.load_calibration('./calibrationdata/left.yaml')
        self.right_calibration = self.load_calibration('./calibrationdata/right.yaml')

        # Publishers for CameraInfo
        self.left_camera_pub = self.create_publisher(CameraInfo, '/left/camera_info', 10)
        self.right_camera_pub = self.create_publisher(CameraInfo, '/right/camera_info', 10)

        # Subscribers for synchronized image topics
        self.left_image_sub = Subscriber(self, Image, '/left/image_raw')
        self.right_image_sub = Subscriber(self, Image, '/right/image_raw')

        # Synchronizer to align image messages
        self.sync = ApproximateTimeSynchronizer(
            [self.left_image_sub, self.right_image_sub], queue_size=10, slop=0.1
        )
        self.sync.registerCallback(self.image_callback)

        # Shutdown control
        self.shutdown_event = Event()
        self.subprocesses = []

        # Start bag play and stereo image processing
        self.start_bag_play()
        self.start_stereo_image_proc()

        # Register shutdown handler
        rclpy.get_default_context().on_shutdown(self.on_shutdown)

        self.get_logger().info("BagStereoNode is running.")

    def load_calibration(self, path):
        """Load calibration data from a YAML file."""
        with open(path, 'r') as file:
            return yaml.safe_load(file)

    def create_camera_info_msg(self, calibration_data, header):
        """Convert calibration data to a CameraInfo message."""
        msg = CameraInfo()
        msg.header = header
        msg.width = calibration_data['image_width']
        msg.height = calibration_data['image_height']
        msg.distortion_model = calibration_data['distortion_model']
        msg.d = calibration_data['distortion_coefficients']['data']
        msg.k = calibration_data['camera_matrix']['data']
        msg.r = calibration_data['rectification_matrix']['data']
        msg.p = calibration_data['projection_matrix']['data']
        return msg

    def image_callback(self, left_msg, right_msg):
        """Callback to publish synchronized CameraInfo messages."""
        # Publish CameraInfo for the left image
        left_info = self.create_camera_info_msg(self.left_calibration, left_msg.header)
        self.left_camera_pub.publish(left_info)

        # Publish CameraInfo for the right image
        right_info = self.create_camera_info_msg(self.right_calibration, right_msg.header)
        self.right_camera_pub.publish(right_info)

        self.get_logger().debug("Published CameraInfo for synchronized images.")

    def start_bag_play(self):
        """Start playing the bag file."""
        def play_bag():
            self.get_logger().info("Playing bag file...")
            proc = subprocess.Popen([
                'ros2', 'bag', 'play', 'ros2.bag2',
                '--remap',
                '/cam0/image_raw:=/left/image_raw',
                '/cam0/camera_info:=/left/camera_info',
                '/cam1/image_raw:=/right/image_raw',
                '/cam1/camera_info:=/right/camera_info',
                '--loop'
            ])
            self.subprocesses.append(proc)
            proc.wait()  # Wait until the process ends

        Thread(target=play_bag, daemon=True).start()

    def start_stereo_image_proc(self):
        """Launch stereo_image_proc."""
        def launch_stereo_proc():
            self.get_logger().info("Launching stereo_image_proc...")
            proc = subprocess.Popen(['ros2', 'launch', 'stereo_image_proc', 'stereo_image_proc.launch.py'])
            self.subprocesses.append(proc)
            proc.wait()  # Wait until the process ends

        Thread(target=launch_stereo_proc, daemon=True).start()

    def on_shutdown(self):
        """Clean up threads and subprocesses on shutdown."""
        self.get_logger().info("Shutting down BagStereoNode...")
        self.shutdown_event.set()  # Signal threads to stop

        # Terminate all subprocesses
        for proc in self.subprocesses:
            if proc.poll() is None:  # Check if the process is still running
                self.get_logger().info(f"Terminating subprocess {proc.pid}...")
                proc.terminate()
                proc.wait()

        self.get_logger().info("All subprocesses terminated. Node shutdown complete.")


def main(args=None):
    rclpy.init(args=args)
    node = BagStereoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl+C detected. Shutting down...")
    finally:
        # Avoid redundant shutdown calls
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()
