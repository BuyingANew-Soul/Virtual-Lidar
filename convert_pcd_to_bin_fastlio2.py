import open3d as o3d
import numpy as np
import struct
import os

def convert_pcd_to_bin(pcd_dir, output_bin):
    pcd_files = sorted([f for f in os.listdir(pcd_dir) if f.endswith('.pcd')])

    with open(output_bin, "wb") as f_bin:
        for pcd_file in pcd_files:
            pcd = o3d.io.read_point_cloud(os.path.join(pcd_dir, pcd_file))
            points = np.asarray(pcd.points)
            for point in points:
                f_bin.write(struct.pack("fff", point[0], point[1], point[2]))  # x, y, z
    print(f"✅ Converted {len(pcd_files)} PCD files to {output_bin}")

convert_pcd_to_bin("/home/zero/VirtualLidar/univ_outdoor_summer_bright_sunny_2019-09-09-14-41-41_raw/vlp/pcd", "/home/zero/VirtualLidar/univ_outdoor_summer_bright_sunny_2019-09-09-14-41-41_raw/vlp/velodyne_hits.bin")
