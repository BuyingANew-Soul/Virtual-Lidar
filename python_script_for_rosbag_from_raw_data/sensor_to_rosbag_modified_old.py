import os
import gc
import rosbag
import rospy
import pandas as pd
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

# ✅ Process GPS Data in Chunks
def write_gps_data(file_path, bag, topic="/gps"):
    gps_chunks = load_large_csv(file_path)
    if gps_chunks is None:
        return
    
    for chunk in gps_chunks:
        for _, row in chunk.iterrows():
            fix = NavSatFix()
            fix.header.stamp = rospy.Time.now()
            fix.latitude = row["lat"]
            fix.longitude = row["long"]
            fix.altitude = row["alt"]
            bag.write(topic, fix, t=fix.header.stamp)
        gc.collect()  # ✅ Free memory after each chunk

# ✅ Process IMU Data in Chunks
def write_imu_data(file_path, bag, topic="/imu"):
    imu_chunks = load_large_csv(file_path)
    if imu_chunks is None:
        return

    for chunk in imu_chunks:
        for _, row in chunk.iterrows():
            imu = Imu()
            imu.header.stamp = rospy.Time.now()
            imu.linear_acceleration.x = row["accel_x"]
            imu.linear_acceleration.y = row["accel_y"]
            imu.linear_acceleration.z = row["accel_z"]
            imu.angular_velocity.x = row["gyro_x"]
            imu.angular_velocity.y = row["gyro_y"]
            imu.angular_velocity.z = row["gyro_z"]
            bag.write(topic, imu, t=imu.header.stamp)
        gc.collect()  # ✅ Free memory after each chunk

# ✅ Process LiDAR Data in Chunks
def write_point_cloud(file_path, bag, topic="/lidar"):
    if not os.path.exists(file_path):
        print(f"⚠️ Warning: {file_path} not found. Skipping...")
        return
    
    with open(file_path, "rb") as f:
        data = []
        while True:
            block = f.read(12)
            if not block:
                break
            x, y, z = list(struct.unpack("fff", block))
            data.append([x, y, z])
            
            # ✅ Write in Chunks of 10,000 Points
            if len(data) >= 10000:
                header = Header()
                header.frame_id = "lidar"
                header.stamp = rospy.Time.now()
                fields = [PointField("x", 0, PointField.FLOAT32, 1),
                          PointField("y", 4, PointField.FLOAT32, 1),
                          PointField("z", 8, PointField.FLOAT32, 1)]
                pcl_msg = pcl2.create_cloud(header, fields, data)
                bag.write(topic, pcl_msg, t=pcl_msg.header.stamp)
                data.clear()
                gc.collect()  # ✅ Free memory

# ✅ Main Function
def main():
    rospy.init_node("rosbag_creator", anonymous=True)

    input_dir = sys.argv[1]
    output_bag = sys.argv[2]
    
    print(f"📂 Creating ROS Bag: {output_bag}")
    bag = rosbag.Bag(output_bag, "w")

    # ✅ Process Data in Chunks
    write_gps_data(os.path.join(input_dir, "gps.csv"), bag)
    write_imu_data(os.path.join(input_dir, "imu.csv"), bag)
    write_point_cloud(os.path.join(input_dir, "lidar.bin"), bag)

    bag.close()
    print("✅ ROS Bag Successfully Created!")

if __name__ == "__main__":
    import sys
    import struct
    sys.exit(main())
