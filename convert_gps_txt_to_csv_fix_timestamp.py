import numpy as np

def convert_gps_txt_to_csv(gps_txt, output_csv):
    """Convert GPS timestamps to floating-point seconds."""
    gps_data = np.loadtxt(gps_txt)

    with open(output_csv, "w") as f:
        for row in gps_data:
            timestamp = float(row[0])  # Already in seconds
            lat, lon, alt = row[1:4]
            f.write(f"{timestamp},1,10,{lat},{lon},{alt},0,0\n")  # Dummy values for mode, satellites, speed

    print(f"✅ GPS data saved as {output_csv}")

convert_gps_txt_to_csv("./GPS_data.txt", "./gps.csv")
