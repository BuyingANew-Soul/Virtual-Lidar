import open3d as o3d
import numpy as np
import struct
import os

def convert_pcd_to_bin(pcd_dir, output_bin):
    pcd_files = sorted([f for f in os.listdir(pcd_dir) if f.endswith('.pcd')])

    with open(output_bin, "wb") as f_bin:
        for pcd_file in pcd_files:
            pcd = o3d.io.read_point_cloud(os.path.join(pcd_dir, pcd_file), format="pcd")
            points = np.asarray(pcd.points)

            # ✅ Try reading intensity if available
            if hasattr(pcd, 'colors') and len(pcd.colors) == len(points):
                intensity = np.mean(np.asarray(pcd.colors), axis=1)  # Use color mean as intensity
            else:
                intensity = np.zeros(len(points))  # Default to 0 if missing

            for i in range(len(points)):
                ring = i % 16  # ✅ Approximate ring ID (VLP-16 has 16 rings)
                f_bin.write(struct.pack("ffffH", points[i][0], points[i][1], points[i][2], intensity[i], ring))


    print(f"✅ Converted {len(pcd_files)} PCD files to {output_bin} (with intensity)")

convert_pcd_to_bin("/home/zero/VirtualLidar/univ_outdoor_summer_bright_sunny_2019-09-09-14-41-41_raw/vlp/pcd",
                    "/home/zero/VirtualLidar/univ_outdoor_summer.bin")

