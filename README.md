# LeGO-LOAM Docker Setup and Usage Guide

## Overview
This repository provides a Docker environment for running **LeGO-LOAM**, a lightweight and ground-optimized LiDAR odometry and mapping framework. The setup includes necessary dependencies such as **ROS Noetic, PCL, and GTSAM**.

## Prerequisites
- **Docker** installed on your system
- **NVIDIA GPU (optional but recommended)**
- **X server setup for GUI applications (e.g., RViz)**

## Setting Up the Docker Environment

### 1. Building the Docker Image
Navigate to the directory containing the `Dockerfile` and run:
```bash
docker build -t ros-noetic-lego-loam .
```

### 2. Creating and Running the Docker Container
Run the script to create and start the container:
```bash
./lego_loam.sh
```
This script:
- Removes any existing container with the same name (`lego-loam`)
- Creates a new container with necessary GPU, USB, and display permissions
- Mounts a directory (`/home/$USER/VirtualLidar/`) for sharing data between host and container. **Change this part in the Dockerfile as you need**

## Running LeGO-LOAM
Inside the container, source the ROS workspace and launch LeGO-LOAM:
```bash
roslaunch lego_loam run.launch
```

## Playing a ROS Bag File
To replay a dataset for testing:
```bash
rosbag play /path/to/.bag --clock /morri/velodyne/velodyne_points:=/velodyne_points /mavros/imu/data:=/imu/data
```

## Extracting Poses
LeGO-LOAM publishes pose information to the following topics:
- `/aft_mapped_to_init` (Final optimized pose)
- `/integrated_to_init` (Integrated odometry result)
- `/odometry` (Initial LiDAR odometry result)

### 1. Check Published Topics
```bash
rostopic list
```

### 2. Save Pose Data to a File
#### Method 1: Using `rostopic echo`
```bash
rostopic echo -b my_rosbag.bag -p /aft_mapped_to_init > poses.csv
```
#### Method 2: Using `rosbag record`
```bash
rosbag record /aft_mapped_to_init /integrated_to_init -O pose_data.bag
```

### 3. Extract Poses from a ROS Bag File
```bash
rosbag play my_rosbag.bag
rostopic echo /aft_mapped_to_init > poses.txt
```

### 4. Using `tf` for Full Trajectory
```bash
rosrun tf tf_echo /map /base_link
```

### 5. Convert Pose Data to Desired Format
Run the Python script `pose_data_transformation.py` to convert the `poses.txt` file into the required format:
```
timestamp, x, y, z, qx, qy, qz, qw
```

## Saving the Global Point Cloud
1. Launch LeGO-LOAM:
```bash
roslaunch lego_loam run.launch
```
2. Play the ROS bag file:
```bash
rosbag play /path/to/.bag --clock \
/morri/velodyne/velodyne_points:=/velodyne_points \
/mavros/imu/data:=/imu/data
```
3. Save the registered cloud as PCD files:
```bash
rosrun pcl_ros pointcloud_to_pcd input:=/registered_cloud _prefix:=scan
```
*This will save `.pcd` files in the directory where the command is executed.*

## Merging Point Cloud Files
To merge all `.pcd` files into a **Global Map**:
```bash
pcl_concatenate_points_pcd scan*.pcd global_map.pcd
```

## Notes
- Ensure the `DISPLAY` environment variable is set correctly when running GUI applications.
- If using GPU acceleration, make sure you have NVIDIA Docker support installed.
- Modify `lego_loam.sh` if paths need to be adjusted for your setup.

---

This guide should help you set up and run LeGO-LOAM efficiently using Docker!

