import numpy as np

def convert_gps_txt_to_csv(gps_txt, output_csv):
    gps_data = np.loadtxt(gps_txt)

    with open(output_csv, "w") as f:
        for row in gps_data:
            timestamp = row[0] / 1e6  # Convert ns to ms
            lat, lon, alt = row[1:4]
            f.write(f"{timestamp},1,10,{lat},{lon},{alt},0,0\n")  # Dummy values for mode, satellites, speed

    print(f"✅ GPS data saved as {output_csv}")

convert_gps_txt_to_csv("/home/zero/VirtualLidar/univ_outdoor_summer_bright_sunny_2019-09-09-14-41-41_raw/GPS_data.txt", "/home/zero/VirtualLidar/univ_outdoor_summer_bright_sunny_2019-09-09-14-41-41_raw/gps.csv")
