import os
import csv
from rosbags.rosbag1 import Reader

bag_dir = 'C:/Users/agori/Documents/MSc 2022/Realsense/Realsense/Pancake2/'  # change to your directory
rows = []

for fname in sorted(os.listdir(bag_dir)):
    if not fname.endswith(".bag"):
        continue
    fpath = os.path.join(bag_dir, fname)
    size = os.path.getsize(fpath)
    try:
        with Reader(fpath) as bag:
            start = bag.start_time / 1e9   # nanoseconds → seconds
            end   = bag.end_time   / 1e9
            duration = end - start
            rows.append({
                "Name":     fname,
                "Start":    round(start, 6),
                "End":      round(end, 6),
                "Size":     size,
                "Duration": round(duration, 6),
            })
    except Exception as e:
        print(f"Error reading {fname}: {e}")

# Save to CSV
with open("bag_metadata.csv", "w", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=header)
    writer.writeheader()
    writer.writerows(rows)

print("\nSaved to bag_metadata.csv")