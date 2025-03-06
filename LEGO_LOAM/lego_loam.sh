#!/bin/bash

# Create a directory for ROS bag files if it doesn't exist
mkdir -p docker_ws

# Allow X server to be accessed from the local machine (for GUI applications like Rviz)
xhost +local:

# Define container name
CONTAINER_NAME="lego-loam"

# Check if the container already exists
if [ "$(docker ps -aq -f name=$CONTAINER_NAME)" ]; then
    echo "Removing existing container: $CONTAINER_NAME"
    docker rm -f $CONTAINER_NAME
fi

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
  /bin/bash -c "source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash && exec bash"

# Attach to the running container
docker exec -it $CONTAINER_NAME bash
