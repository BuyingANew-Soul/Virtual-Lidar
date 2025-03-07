import yaml

# Load pose.txt
pose_file = "./pose.txt"

poses = []
with open(pose_file, "r") as f:
    docs = f.read().split("---")  # Each pose is separated by "---"
    for doc in docs:
        if doc.strip():
            data = yaml.safe_load(doc)
            timestamp = data["header"]["stamp"]["secs"] + data["header"]["stamp"]["nsecs"] * 1e-9
            x = data["pose"]["pose"]["position"]["x"]
            y = data["pose"]["pose"]["position"]["y"]
            z = data["pose"]["pose"]["position"]["z"]
            qx = data["pose"]["pose"]["orientation"]["x"]
            qy = data["pose"]["pose"]["orientation"]["y"]
            qz = data["pose"]["pose"]["orientation"]["z"]
            qw = data["pose"]["pose"]["orientation"]["w"]
            poses.append([timestamp, x, y, z, qx, qy, qz, qw])

# Save poses in CSV format
import numpy as np
np.savetxt("./poses.csv", poses, delimiter=",", header="timestamp,x,y,z,qx,qy,qz,qw", comments='')
print("Poses extracted and saved to poses.csv")
