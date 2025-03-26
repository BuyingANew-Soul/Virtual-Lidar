import numpy as np
import pandas as pd
import open3d as o3d
from scipy.spatial.transform import Rotation
import argparse

def load_exact_pose(timestamp, pose_file):
    """Load the exact SLAM pose for the given timestamp."""
    pose_df = pd.read_csv(pose_file)
    pose_df["timestamp"] = pose_df["timestamp"].astype(float)

    # Find the row where the timestamp matches exactly
    pose_row = pose_df[np.isclose(pose_df["timestamp"], timestamp, atol=1e-6)]
    
    if pose_row.empty:
        raise ValueError(f"❌ No exact match found for timestamp {timestamp} in {pose_file}")

    # Extract translation (t) and quaternion (q)
    t = np.array([pose_row["t_x"].values[0], pose_row["t_y"].values[0], pose_row["t_z"].values[0]])
    q = np.array([pose_row["q_x"].values[0], pose_row["q_y"].values[0], pose_row["q_z"].values[0], pose_row["q_w"].values[0]])  # (x, y, z, w)

    return t, q

def quaternion_to_rotation_matrix(q):
    """Convert a quaternion (x, y, z, w) to a 3x3 rotation matrix."""
    q = np.roll(q, shift=-1)  # Convert (x, y, z, w) → (w, x, y, z) for scipy
    R = Rotation.from_quat(q).as_matrix()
    return R

def transform_point_cloud(pcd_file, t, q, output_file):
    """Apply the transformation (rotation + translation) to the point cloud."""
    # Load point cloud
    pcd = o3d.io.read_point_cloud(pcd_file)
    points = np.asarray(pcd.points)

    # Convert quaternion to rotation matrix
    R = quaternion_to_rotation_matrix(q)

    # Compute inverse transformation
    R_inv = R.T  # Inverse of rotation matrix is its transpose
    PLiDAR = (points - t) @ R_inv  # Apply transformation

    # Save transformed point cloud
    pcd.points = o3d.utility.Vector3dVector(PLiDAR)
    o3d.io.write_point_cloud(output_file, pcd)
    print(f"✅ Transformed point cloud saved to: {output_file}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Apply SLAM to LiDAR coordinate transformation.")
    parser.add_argument("timestamp", type=float, help="Exact timestamp for the pose.")
    parser.add_argument("pcd_file", type=str, help="Path to the .pcd file.")
    parser.add_argument("--pose_file", type=str, default="slam_poses.csv", help="Path to slam_poses.csv")
    parser.add_argument("--output_file", type=str, default="transformed.pcd", help="Path to save the transformed .pcd file.")

    args = parser.parse_args()

    # Load exact pose data
    try:
        t, q = load_exact_pose(args.timestamp, args.pose_file)

        # Transform the point cloud
        transform_point_cloud(args.pcd_file, t, q, args.output_file)
    except ValueError as e:
        print(e)


## Usage
# python transform_slam_to_lidar.py 1556192427.090186 lidar_scan.pcd --output_file lidar_frame.pcd
#                                    timestamp

# python3 SLAM_coor_to_LIdar_transform.py 1556192444.8449538 ../1_laser_scan_to_SLAM_coordinate_transformation/laser_scan_in_SLAM_coor.pcd --output_file 1556192444.8449538.pcd
# python3 SLAM_coor_to_LIdar_transform.py 1556192445.245493 ../1_laser_scan_to_SLAM_coordinate_transformation/laser_scan_in_SLAM_coor.pcd --output_file 1556192445.245493.pcd
# python3 SLAM_coor_to_LIdar_transform.py 1556192495.172195 ../1_laser_scan_to_SLAM_coordinate_transformation/laser_scan_in_SLAM_coor.pcd --output_file 1556192495.172195.pcd
# python3 SLAM_coor_to_LIdar_transform.py 1556192534.306719 ../1_laser_scan_to_SLAM_coordinate_transformation/laser_scan_in_SLAM_coor.pcd --output_file 1556192534.306719.pcd
# python3 SLAM_coor_to_LIdar_transform.py 1556192538.643805 ../1_laser_scan_to_SLAM_coordinate_transformation/laser_scan_in_SLAM_coor.pcd --output_file 1556192538.643805.pcd
