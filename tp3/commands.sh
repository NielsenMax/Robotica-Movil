# Convert calibration from ros1 to ros2
rosbags-convert --src cam_checkerboard.bag --dst ros2.bag2 --dst-typestore latest 

# Run calibration
ros2 run camera_calibration cameracalibrator --size 7x6 --square 0.06 right:=/cam1/image_raw left:=/cam0/image_raw left_camera:=/cam0 right_camera:=/cam1

# Play rosbag
ros2 bag play ros2.bag2/