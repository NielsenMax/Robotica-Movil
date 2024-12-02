# Use a ROS2 image as the base
FROM osrf/ros:humble-desktop

# Instalar paquetes requeridos
RUN apt-get update && apt-get install -y \
    screen \
    ros-humble-rviz2 \
    ros-humble-stereo-image-proc \
    ros-humble-image-pipeline \
    python3-pip \
    aria2 \
    python3-pandas \
    vim
    #&& rm -rf /var/lib/apt/lists/*
        
# Install rosbag conversion tool (assuming it's available via pip or apt)
RUN pip install rosbags

# Use aria2c to download the cam_checkerboard.bag dataset with maximum threads
#RUN aria2c -x 4 -s 4 http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/calibration_datasets/cam_checkerboard/cam_checkerboard.bag -d /root/

COPY ./Checkboard/cam_checkerboard.bag /root/cam_checkerboard.bag
#COPY ./MH1 /root/ws/MH1

# Convert the bag file to ROS2 format
#RUN rosbags-convert --src /root/cam_checkerboard.bag --dst /root/ws/ros2.bag2 --dst-typestore latest

COPY ros2.bag2 /root/ws/ros2.bag2

# Copiar la carpeta scripts al contenedor
COPY ./calibrationdata /root/ws/calibrationdata
COPY ./scripts /root/ws/scripts
COPY ./bag_stereo_package/ /root/ws/bag_stereo_package

# Set the working directory
WORKDIR /root/ws

# Build your workspace if necessary
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build && colcon test"

# Allow X11 access for GUI applications (like RViz)
ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /root/ws/install/setup.bash" >> ~/.bashrc
RUN echo "alias play=\"ros2 run bag_stereo_package bag_stereo_node\"" >> ~/.bashrc
RUN echo "alias match=\"ros2 run bag_stereo_package matcher_node\"" >> ~/.bashrc
# Dar permisos de ejecución al script
#RUN chmod +x /root/ws/scripts/entrypoint.sh

# Configurar el script como el punto de entrada
#ENTRYPOINT ["/root/ws/scripts/entrypoint.sh"]