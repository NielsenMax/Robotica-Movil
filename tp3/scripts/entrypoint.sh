#!/bin/bash
set -e

# Fuente de los entornos de ROS2
source /opt/ros/humble/setup.bash
source /root/ws/install/setup.bash

# Ejecutar procesos en segundo plano
python3 /root/ws/scripts/camera_info.py & 
ros2 launch stereo_image_proc stereo_image_proc.launch.py &
ros2 bag play /root/ws/ros2.bag2 \
    --remap /cam0/image_raw:=/left/image_raw \
    --remap /cam0/camera_info:=/left/camera_info \
    --remap /cam1/image_raw:=/right/image_raw \
    --remap /cam1/camera_info:=/right/camera_info \
    --loop &

python3 /root/ws/scripts/matcher.py &  # Asegúrate de que esta línea no esté comentada

# Mantener el contenedor en ejecución
wait
