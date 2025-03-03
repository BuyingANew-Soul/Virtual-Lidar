import numpy as np

def convert_imu_txt_to_csv(imu_ori_file, imu_vel_file, output_csv):
    imu_ori = np.loadtxt(imu_ori_file)
    imu_vel = np.loadtxt(imu_vel_file)

    with open(output_csv, "w") as f:
        for i in range(min(len(imu_ori), len(imu_vel))):
            timestamp = imu_ori[i, 0] / 1e6  # Convert ns to ms
            qx, qy, qz, qw = imu_ori[i, 1:5]
            vx, vy, vz, ax, ay, az = imu_vel[i, 1:7]
            f.write(f"{timestamp},{vx},{vy},{vz},{ax},{ay},{az},{qx},{qy},{qz},{qw}\n")

    print(f"✅ IMU data saved as {output_csv}")

convert_imu_txt_to_csv("/home/zero/VirtualLidar/univ_outdoor_summer_bright_sunny_2019-09-09-14-41-41_raw/imu_data_orientation.txt",
                        "/home/zero/VirtualLidar/univ_outdoor_summer_bright_sunny_2019-09-09-14-41-41_raw/imu_data_velocity.txt",
                        "/home/zero/VirtualLidar/univ_outdoor_summer_bright_sunny_2019-09-09-14-41-41_raw/ms25.csv")
