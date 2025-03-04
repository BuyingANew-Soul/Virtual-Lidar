#!/bin/bash
mkdir docker_ws
# Script to run ROS Kinetic with GUI support in Docker

# Allow X server to be accessed from the local machine
xhost +local:

# Container name
CONTAINER_NAME="lego-loam"

# Run the Docker container
docker run -itd \
  --name=$CONTAINER_NAME \
  --gpus all \
  --user root \
  --network host \
  --ipc=host \
  -v /home/$USER/VirtualLidar/:/home/root/docker_ws \
  --privileged \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/etc/localtime:/etc/localtime:ro" \
  -v /dev/bus/usb:/dev/bus/usb \
  --device=/dev/dri \
  --group-add video \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --env="DISPLAY=$DISPLAY" \
  ros-noetic-lego-loam:latest \
  /bin/bash