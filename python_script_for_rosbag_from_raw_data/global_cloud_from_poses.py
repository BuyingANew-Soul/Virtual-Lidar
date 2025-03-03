import os
import argparse
import numpy as np
import open3d as o3d

def load_poses_kitti(file_path):
    """Loads poses from KITTI format (12 values per line)."""
    poses = np.loadtxt(file_path)  # (N, 12)
    pose_matrices = []
    
    for pose_vec in poses:
        T = np.eye(4)
        T[:3, :4] = pose_vec.reshape(3, 4)  # Convert row to 4x4 matrix
        pose_matrices.append(T)

    return pose_matrices

def load_poses_npy(file_path):
    """Loads poses from a NumPy (.npy) file."""
    return np.load(file_path)  # Returns (N, 4, 4)

def transform_point_cloud(pcd, transform):
    """Applies a transformation matrix to a point cloud."""
    points = np.asarray(pcd.points)  # (N, 3)
    points_h = np.hstack((points, np.ones((points.shape[0], 1))))  # (N, 4) Homogeneous
    transformed_points = (transform @ points_h.T).T[:, :3]  # Apply transformation

    transformed_pcd = o3d.geometry.PointCloud()
    transformed_pcd.points = o3d.utility.Vector3dVector(transformed_points)

    return transformed_pcd

def generate_global_map(data_dir, pose_file, output_file, use_kitti):
    """Generates a global point cloud map using LiDAR scans and estimated poses."""
    print(f"Loading poses from {'KITTI' if use_kitti else 'NumPy'} format...")

    poses = load_poses_kitti(pose_file) if use_kitti else load_poses_npy(pose_file)
    global_map = o3d.geometry.PointCloud()

    lidar_files = sorted([f for f in os.listdir(data_dir) if f.endswith('.pcd')])

    if len(lidar_files) != len(poses):
        print(f"Warning: Number of LiDAR scans ({len(lidar_files)}) does not match number of poses ({len(poses)}).")

    print(f"Processing {len(lidar_files)} frames...")
    
    for i, (lidar_file, pose) in enumerate(zip(lidar_files, poses)):
        pcd_path = os.path.join(data_dir, lidar_file)
        pcd = o3d.io.read_point_cloud(pcd_path)

        transformed_pcd = transform_point_cloud(pcd, pose)
        global_map += transformed_pcd  # Accumulate into the global map

        if i % 50 == 0:
            print(f"Processed {i}/{len(lidar_files)} frames...")

    # Optional: Voxel Downsampling
    global_map = global_map.voxel_down_sample(voxel_size=0.1)

    # Save and visualize
    o3d.io.write_point_cloud(output_file, global_map)
    print(f"Global map saved to {output_file}")
    o3d.visualization.draw_geometries([global_map])

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate a global map from LiDAR scans and estimated poses.")
    parser.add_argument("--data_dir", required=True, help="Directory containing LiDAR .pcd files")
    parser.add_argument("--pose_dir", required=True, help="Path to pose file (pcd_poses.npy or pcd_poses_kitti.txt)")
    parser.add_argument("--output", required=True, help="Output file for the global map (.pcd)")
    parser.add_argument("--kitti", action="store_true", help="Use KITTI format for poses")

    args = parser.parse_args()
    generate_global_map(args.data_dir, args.pose_dir, args.output, args.kitti)
