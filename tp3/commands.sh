# Convert calibration from ros1 to ros2
rosbags-convert --src cam_checkerboard.bag --dst ros2.bag2 --dst-typestore latest 

# Run calibration
ros2 run camera_calibration cameracalibrator --size 7x6 --square 0.06 right:=/cam1/image_raw left:=/cam0/image_raw left_camera:=/cam0 right_camera:=/cam1

# Play rosbag
ros2 bag play ros2.bag2/

#Rectification
ros2 launch stereo_image_proc stereo_image_proc.launch.py

ros2 bag play ros2.bag2 --remap /cam0/image_raw:=/left/image_raw \
/cam0/camera_info:=/left/camera_info \
/cam1/image_raw:=/right/image_raw \
/cam1/camera_info:=/right/camera_info

python3 ./camera_info.py

ros2 run image_view image_view image:=/left/image_rect_color

#Features
#Correr todos los anteriores
python3 ./features.py