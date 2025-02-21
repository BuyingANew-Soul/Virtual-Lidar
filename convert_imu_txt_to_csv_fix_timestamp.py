import numpy as np

def convert_imu_txt_to_csv(imu_ori_file, imu_vel_file, output_csv):
    """Convert IMU orientation and velocity data to CSV with timestamps in seconds."""
    imu_ori = np.loadtxt(imu_ori_file)
    imu_vel = np.loadtxt(imu_vel_file)

    with open(output_csv, "w") as f:
        for i in range(min(len(imu_ori), len(imu_vel))):
            timestamp = imu_ori[i, 0] / 1e9  # Convert nanoseconds to seconds
            qx, qy, qz, qw = imu_ori[i, 1:5]
            vx, vy, vz, ax, ay, az = imu_vel[i, 1:7]
            f.write(f"{timestamp},{vx},{vy},{vz},{ax},{ay},{az},{qx},{qy},{qz},{qw}\n")

    print(f"✅ IMU data saved as {output_csv}")

convert_imu_txt_to_csv("./imu_data_orientation.txt",
                        ".imu_data_velocity.txt",
                        "./ms25.csv")
