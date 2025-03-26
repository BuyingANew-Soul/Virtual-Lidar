#!/usr/bin/env python

import rosbag
import csv
import numpy as np
from scipy.spatial.transform import Rotation

# ROS Bag path
bag_path = "pose_without_imu_data.bag"
topic = "/aft_mapped_to_init"  # Change to "/integrated_to_init" if needed
output_csv = "slam_poses.csv"

# Open ROS bag and extract pose data
bag = rosbag.Bag(bag_path, "r")

poses = []
for topic, msg, t in bag.read_messages(topics=[topic]):
    timestamp = msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9
    
    # Translation (position)
    t_x = msg.pose.pose.position.x
    t_y = msg.pose.pose.position.y
    t_z = msg.pose.pose.position.z
    
    # Rotation (quaternion)
    q_x = msg.pose.pose.orientation.x
    q_y = msg.pose.pose.orientation.y
    q_z = msg.pose.pose.orientation.z
    q_w = msg.pose.pose.orientation.w
    
    # Store data
    poses.append([timestamp, t_x, t_y, t_z, q_x, q_y, q_z, q_w])

bag.close()
print(f"Extracted {len(poses)} poses from {topic}")

# Save to CSV
with open(output_csv, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["timestamp", "t_x", "t_y", "t_z", "q_x", "q_y", "q_z", "q_w"])
    writer.writerows(poses)

print(f"✅ Poses saved to {output_csv}")
