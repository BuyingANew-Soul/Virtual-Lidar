# Use ROS Noetic with Ubuntu 20.04 as the base image
FROM ros:noetic-ros-base-focal

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=America/New_York

# Install ROS perception packages and essential dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-perception=1.5.0-1* \
    ros-noetic-rviz \
    libusb-1.0-0-dev \
    libparmetis-dev \
    build-essential \
    cmake \
    git \
    wget \
    unzip \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /home/root

# Install PCL 1.12.1
RUN wget https://github.com/PointCloudLibrary/pcl/archive/refs/tags/pcl-1.12.1-rc1.tar.gz && \
    tar -xzf pcl-1.12.1-rc1.tar.gz && \
    cd pcl-pcl-1.12.1-rc1 && \
    mkdir release && cd release && \
    cmake -DCMAKE_BUILD_TYPE=None -DCMAKE_INSTALL_PREFIX=/usr \
          -DBUILD_GPU=ON -DBUILD_apps=ON -DBUILD_examples=ON .. && \
    make -j$(nproc) && \
    make install && \
    cd /home/root && rm -rf pcl-pcl-1.12.1-rc1 pcl-1.12.1-rc1.tar.gz

# Install GTSAM 4.1.1
RUN wget https://github.com/borglab/gtsam/archive/refs/tags/4.1.1.tar.gz && \
    tar -xzf 4.1.1.tar.gz && \
    cd gtsam-4.1.1 && \
    mkdir build && cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install && \
    cd /home/root && rm -rf gtsam-4.1.1 4.1.1.tar.gz

# Setup Catkin workspace and install LeGO-LOAM
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    mkdir -p ~/catkin_ws/src && \
    cd ~/catkin_ws/src && \
    git clone https://github.com/Rotvie/LeGO-LOAM && \
    cd ~/catkin_ws && \
    catkin_make -j1 && \
    echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc"


# Set entrypoint to automatically source ROS setup files
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash && exec bash"]
