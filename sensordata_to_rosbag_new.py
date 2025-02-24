import os
import gc
import sys
import struct
import numpy as np
import pandas as pd
import rosbag
import rospy
from tqdm import tqdm
from sensor_msgs.msg import Imu, NavSatFix, PointCloud2, PointField
import sensor_msgs.point_cloud2 as pcl2
from std_msgs.msg import Header



# ✅ Function to Load Large CSV Files in Chunks (Memory Efficient)
def load_large_csv(file_path, chunk_size=10000):
    if not os.path.exists(file_path):
        print(f"⚠️ Warning: {file_path} not found. Skipping...")
        return None
    return pd.read_csv(file_path, chunksize=chunk_size)

# ✅ Function to Find Nearest Timestamp in Sorted Arrays
def find_nearest_time(target_time, time_array):
    index = np.abs(time_array - target_time).argmin()
    return time_array[index], index

# ✅ Function to Linearly Interpolate Between Two Timestamps
def linear_interpolate(target_time, time1, time2, value1, value2):
    if time1 == time2:
        return value1  # Avoid division by zero
    ratio = (target_time - time1) / (time2 - time1)
    return value1 + ratio * (value2 - value1)

# ✅ Process GPS Data in Chunks
def write_gps_data(file_path, bag, topic="/gps"):
    gps_chunks = load_large_csv(file_path)
    if gps_chunks is None:
        return

    gps_data = []
    for chunk in gps_chunks:
        gps_data.append(chunk.values)
    gps_data = np.vstack(gps_data)  # Convert list of chunks to numpy array
    gps_times = gps_data[:, 0]  # First column is timestamp (already in seconds)

    print(f"📂 Writing {len(gps_times)} GPS messages at full rate")

    for gps_entry in tqdm(gps_data, desc="Processing GPS Data", mininterval=1.0):
        fix = NavSatFix()
        fix.header.stamp = rospy.Time.from_sec(gps_entry[0])  # Use actual GPS timestamp
        fix.latitude = gps_entry[3]
        fix.longitude = gps_entry[4]
        fix.altitude = gps_entry[5]
        bag.write(topic, fix, t=fix.header.stamp)

    print("✅ GPS data written at full rate!")


# ✅ Process IMU Data in Chunks
def write_imu_data(file_path, bag, topic="/imu/data"):
    imu_chunks = load_large_csv(file_path)
    if imu_chunks is None:
        return

    imu_data = []
    for chunk in imu_chunks:
        imu_data.append(chunk.values)
    imu_data = np.vstack(imu_data)  # Convert list of chunks to numpy array
    imu_times = imu_data[:, 0]  # First column is timestamp (already in seconds)

    print(f"📂 Writing {len(imu_times)} IMU messages at full rate")

    for imu_entry in tqdm(imu_data, desc="Processing IMU Data", mininterval=1.0):
        imu = Imu()
        imu.header.stamp = rospy.Time.from_sec(imu_entry[0])  # Use actual IMU timestamp
        imu.linear_acceleration.x = imu_entry[1]
        imu.linear_acceleration.y = imu_entry[2]
        imu.linear_acceleration.z = imu_entry[3]
        imu.angular_velocity.x = imu_entry[4]
        imu.angular_velocity.y = imu_entry[5]
        imu.angular_velocity.z = imu_entry[6]
        bag.write(topic, imu, t=imu.header.stamp)

    print("✅ IMU data written at full rate!")




def write_lidar_data(pcd_txt, bin_file, bag, topic="/velodyne_points", batch_size=10000):
    """Efficiently read LiDAR timestamps from `pcd.txt` and point cloud data from `velodyne_hits.bin` and write to ROS bag in batches."""

    if not os.path.exists(pcd_txt):
        print(f"⚠️ Warning: {pcd_txt} not found. Skipping...")
        return []

    if not os.path.exists(bin_file):
        print(f"⚠️ Warning: {bin_file} not found. Skipping...")
        return []

    # Load timestamps from pcd.txt
    pcd_data = pd.read_csv(pcd_txt, delimiter=" ", skiprows=2, header=None, names=["timestamp", "filename"])
    lidar_times = pcd_data["timestamp"].values.astype(float)

    print(f"📂 Loading {len(lidar_times)} LiDAR timestamps from {pcd_txt}")

    # Open LiDAR .bin file
    with open(bin_file, "rb") as f_bin:
        total_points = 0
        for i, lidar_time in enumerate(tqdm(lidar_times, desc="Processing LiDAR Data", mininterval=5.0)):
            timestamp = rospy.Time.from_sec(lidar_time)
            
            # Read batch of points
            batch_points = []
            while len(batch_points) < batch_size:
                block = f_bin.read(12)  # Read 3 floats (x, y, z)
                if not block:
                    break
                x, y, z = struct.unpack("fff", block)
                batch_points.append([x, y, z, lidar_time])  # Append with timestamp

            if batch_points:
                total_points += len(batch_points)
                batch_points = np.array(batch_points, dtype=np.float32)  # Convert to NumPy array for efficiency

                # Create PointCloud2 message
                header = Header()
                header.frame_id = "velodyne"
                header.stamp = timestamp

                fields = [
                    PointField("x", 0, PointField.FLOAT32, 1),
                    PointField("y", 4, PointField.FLOAT32, 1),
                    PointField("z", 8, PointField.FLOAT32, 1),
                    PointField("time", 12, PointField.FLOAT32, 1),  # Ensure `time` field exists
                ]
                pcl_msg = pcl2.create_cloud(header, fields, batch_points)
                bag.write(topic, pcl_msg, t=timestamp)

            # Force garbage collection after each batch
            gc.collect()

    print(f"✅ Processed {len(lidar_times)} LiDAR frames, {total_points} total points")
    return lidar_times



def main():
    rospy.init_node("rosbag_creator", anonymous=True)

    input_dir = sys.argv[1]
    output_bag = sys.argv[2]

    print(f"📂 Creating ROS Bag: {output_bag}")
    bag = rosbag.Bag(output_bag, "w")

    # ✅ Read LiDAR timestamps & point cloud data from .bin
    lidar_times = write_lidar_data(os.path.join(input_dir, "pcd.txt"), 
                                   os.path.join(input_dir, "velodyne_hits.bin"), 
                                   bag)

    # ✅ Process GPS and IMU data at full rate (not downsampled!)
    write_gps_data(os.path.join(input_dir, "gps.csv"), bag)
    write_imu_data(os.path.join(input_dir, "ms25.csv"), bag)

    bag.close()
    print("✅ ROS Bag Successfully Created!")

if __name__ == "__main__":
    sys.exit(main())

