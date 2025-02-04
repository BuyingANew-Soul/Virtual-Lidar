import argparse
import os
import numpy as np
import open3d as o3d
from natsort import natsorted

def read_poses(poses_path):
    """Read poses from a text file."""
    poses = []
    with open(poses_path, 'r') as f:
        for line in f:
            matrix_values = list(map(float, line.strip().split()))
            pose = np.array(matrix_values).reshape(3, 4)
            pose = np.vstack([pose, [0, 0, 0, 1]])  # Convert to 4x4 matrix
            poses.append(pose)
    return poses

def transform_point_clouds(pcd_dir, poses):
    """Transform point clouds to a common coordinate system."""
    pcd_files = natsorted([f for f in os.listdir(pcd_dir) if f.endswith('.pcd')])

    if len(pcd_files) != len(poses):
        raise ValueError("Number of point clouds and poses must match.")

    global_pcd = o3d.geometry.PointCloud()

    for i, pcd_file in enumerate(pcd_files):
        pcd_path = os.path.join(pcd_dir, pcd_file)
        pcd = o3d.io.read_point_cloud(pcd_path)

        # Apply the transformation
        pcd.transform(poses[i])

        # Merge into the global point cloud
        global_pcd += pcd
        
        if i > 500:
            break

    global_pcd.paint_uniform_color([0.0, 0.0, 1.0])
    global_pcd = global_pcd.voxel_down_sample(voxel_size=0.1)
    
    return global_pcd

def main(pcd_dir, poses_path):
    """Main function to process point clouds and poses."""
    # Read poses
    poses = read_poses(poses_path)

    # Transform point clouds
    global_pcd = transform_point_clouds(pcd_dir, poses)

    # Save the global point cloud
    output_path = "global_point_cloud.ply"
    o3d.io.write_point_cloud(output_path, global_pcd)
    print(f"Global point cloud saved to {output_path}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Transform point clouds using poses and merge them into a global point cloud.")
    parser.add_argument("pcd_dir", type=str, help="Directory containing PLY point clouds.")
    parser.add_argument("poses_path", type=str, help="Path to the text file with 3x4 transformation matrices.")
    
    args = parser.parse_args()
    main(args.pcd_dir, args.poses_path)
