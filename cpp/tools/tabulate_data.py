#!/usr/bin/env python3
"""
tabulate_data.py — build a one-row-per-recording metadata table from a
rs_marker dataset that has already been through collect_plys.py's default
mode (i.e. "<src>_raw/" and "<src>_collected/{ply,detections,imu}/" exist
next to <src>).

Reads each .bag's header via the `rosbags` package (no pyrealsense2 or ROS
install needed — same approach as python-3DRecon/realsense/metadata_gen.py)
to get timing, device, per-stream frame count/FPS/resolution and IMU sample
counts, then cross-references <src>_collected/ to report which of
marker_csv/ply_file/imu_csv exist for that recording (NaN if missing).

Requires: rosbags
    pip install --break-system-packages rosbags

Usage:
    python3 tools/tabulate_data.py /media/aru/Seagate/roughness/20260714
    python3 tools/tabulate_data.py /media/aru/Seagate/roughness/20260714 --out /tmp/meta.csv
"""

import argparse
import csv
import sys
from pathlib import Path

from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore

TYPESTORE = get_typestore(Stores.ROS1_NOETIC)
NaN = "NaN"

COLUMNS = [
    "Name", "Start", "End", "Duration", "Size",
    "Device", "Serial", "Firmware",
    "Depth_Frames", "Color_Frames", "Depth_FPS", "Color_FPS",
    "Depth_Res", "Color_Res",
    "Accel_Samples", "Gyro_Samples",
    "marker_csv", "ply_file", "imu_csv",
    "num_detections", "num_unique_markers",
]


def find_topic(conns, suffix):
    return next((t for t in conns if t.endswith(suffix)), None)


def read_device_info(bag, conns):
    topic = find_topic(conns, "/device_0/info")
    info = {}
    if topic is None:
        return info
    for c, _, rawdata in bag.messages(connections=[conns[topic]]):
        kv = TYPESTORE.deserialize_ros1(rawdata, c.msgtype)
        info[kv.key] = kv.value
    return info


def read_resolution(bag, conns, suffix):
    topic = find_topic(conns, suffix)
    if topic is None:
        return None
    for c, _, rawdata in bag.messages(connections=[conns[topic]]):
        msg = TYPESTORE.deserialize_ros1(rawdata, c.msgtype)
        return f"{msg.width}x{msg.height}"
    return None


def msg_count(conns, suffix):
    topic = find_topic(conns, suffix)
    return conns[topic].msgcount if topic else None


def read_bag_row(bag_path):
    row = {col: NaN for col in COLUMNS}
    row["Name"] = bag_path.name
    row["Size"] = bag_path.stat().st_size

    with Reader(bag_path) as bag:
        conns = {c.topic: c for c in bag.connections}

        start = bag.start_time / 1e9
        end = bag.end_time / 1e9
        duration = end - start
        row["Start"] = round(start, 6)
        row["End"] = round(end, 6)
        row["Duration"] = round(duration, 6)

        info = read_device_info(bag, conns)
        row["Device"] = info.get("Name", NaN)
        row["Serial"] = info.get("Serial Number", NaN)
        row["Firmware"] = info.get("Firmware Version", NaN)

        depth_frames = msg_count(conns, "Depth_0/image/data")
        color_frames = msg_count(conns, "Color_0/image/data")
        row["Depth_Frames"] = depth_frames if depth_frames is not None else NaN
        row["Color_Frames"] = color_frames if color_frames is not None else NaN
        row["Depth_FPS"] = round(depth_frames / duration, 2) if depth_frames and duration > 0 else NaN
        row["Color_FPS"] = round(color_frames / duration, 2) if color_frames and duration > 0 else NaN

        row["Depth_Res"] = read_resolution(bag, conns, "Depth_0/info/camera_info") or NaN
        row["Color_Res"] = read_resolution(bag, conns, "Color_0/info/camera_info") or NaN

        accel = msg_count(conns, "Accel_0/imu/data")
        gyro = msg_count(conns, "Gyro_0/imu/data")
        row["Accel_Samples"] = accel if accel is not None else NaN
        row["Gyro_Samples"] = gyro if gyro is not None else NaN

    return row


def count_detections(csv_path):
    n_rows = 0
    marker_ids = set()
    with open(csv_path, newline="") as f:
        for r in csv.DictReader(f):
            n_rows += 1
            marker_ids.add(r["marker_id"])
    return n_rows, len(marker_ids)


def parse_args():
    p = argparse.ArgumentParser(description="Tabulate rs_marker bag metadata + collected file presence into one CSV")
    p.add_argument("src", type=Path,
                    help="Recordings folder previously passed to collect_plys.py "
                         "(expects <src>_raw/ and <src>_collected/{ply,detections,imu}/ to exist)")
    p.add_argument("--out", type=Path, default=None,
                    help="Output CSV path (default: <src>_metadata.csv, next to src)")
    return p.parse_args()


def main():
    args = parse_args()

    raw_dir = args.src.parent / f"{args.src.name}_raw"
    collected_dir = args.src.parent / f"{args.src.name}_collected"
    ply_dir = collected_dir / "ply"
    detections_dir = collected_dir / "detections"
    imu_dir = collected_dir / "imu"

    if not raw_dir.is_dir():
        sys.exit(f"Error: {raw_dir} not found — run collect_plys.py in default mode on {args.src} first")

    out_path = args.out or args.src.parent / f"{args.src.name}_metadata.csv"

    bag_files = sorted(raw_dir.glob("*.bag"))
    if not bag_files:
        sys.exit(f"No .bag files found in {raw_dir}")

    rows = []
    for bag_path in bag_files:
        stem = bag_path.stem
        try:
            row = read_bag_row(bag_path)
        except Exception as e:
            print(f"Warning: failed to read {bag_path.name}: {e}", file=sys.stderr)
            row = {col: NaN for col in COLUMNS}
            row["Name"] = bag_path.name
            row["Size"] = bag_path.stat().st_size

        marker_csv = detections_dir / f"{stem}_detections.csv"
        ply_file = ply_dir / f"{stem}.ply"
        imu_csv = imu_dir / f"{stem}_imu.csv"

        row["marker_csv"] = marker_csv.name if marker_csv.exists() else NaN
        row["ply_file"] = ply_file.name if ply_file.exists() else NaN
        row["imu_csv"] = imu_csv.name if imu_csv.exists() else NaN

        if marker_csv.exists():
            try:
                n_rows, n_unique = count_detections(marker_csv)
                row["num_detections"] = n_rows
                row["num_unique_markers"] = n_unique
            except Exception as e:
                print(f"Warning: failed to read {marker_csv.name}: {e}", file=sys.stderr)

        rows.append(row)

    with open(out_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=COLUMNS)
        writer.writeheader()
        writer.writerows(rows)

    print(f"Wrote {len(rows)} rows to {out_path}")


if __name__ == "__main__":
    main()
