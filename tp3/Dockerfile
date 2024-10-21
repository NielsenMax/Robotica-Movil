# Use a ROS2 image as the base
FROM osrf/ros:humble-desktop

# Install required packages
RUN apt-get update

RUN apt-get install -y \
    ros-humble-rviz2 \
    ros-humble-stereo-image-proc \
    python3-pip \
    aria2 \
    && rm -rf /var/lib/apt/lists/*

# Install rosbag conversion tool (assuming it's available via pip or apt)
RUN pip install rosbags

# Use aria2c to download the cam_checkerboard.bag dataset with maximum threads
RUN aria2c -x 4 -s 4 http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/calibration_datasets/cam_checkerboard/cam_checkerboard.bag -d /root/

# Convert the bag file to ROS2 format
RUN rosbags-convert --src /root/cam_checkerboard.bag --dst /root/ws/ros2.bag2 --dst-typestore latest

# Copy your ROS2 workspace and scripts into the container
#COPY ./my_ros2_workspace /root/my_ros2_workspace
COPY ./calibrationdata /root/ws/calibrationdata
COPY ./matcher.py /root/ws/matcher.py

# Set the working directory
WORKDIR /root/ws

# Build your workspace if necessary
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# Allow X11 access for GUI applications (like RViz)
ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1

# Default command to launch everything
CMD ["bash", "-c", "\
    source /opt/ros/humble/setup.bash && \
    source install/setup.bash && \
    ros2 launch stereo_image_proc stereo_image_proc.launch.py & \
    ros2 bag play ros2.bag2 --remap /cam0/image_raw:=/left/image_raw /cam0/camera_info:=/left/camera_info /cam1/image_raw:=/right/image_raw /cam1/camera_info:=/right/camera_info & \
    python3 matcher.py & \
    rviz2"]
