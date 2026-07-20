#!/usr/bin/env python3
"""
marker_points.py — batch-extract 3D marker positions from a folder of
RealSense .bag recordings + their AprilTag detection CSVs, and save one
output file per recording (in the same depth optical frame as your
reconstructed point clouds) for overlay/QC in CloudCompare, Open3D, etc.,
or for downstream analysis.

Usage:
    python3 marker_points.py <raw_dir> <collected_dir> [--format ply|csv]

<raw_dir>        folder of raw .bag recordings, e.g. .../20260714_raw
<collected_dir>  collect_plys.py output folder, e.g. .../20260714_collected
                 (must contain a detections/ subfolder of *_detections.csv)

Each <collected_dir>/detections/<stem>_detections.csv is paired with
<raw_dir>/<stem>.bag (matched by timestamp stem). Recordings with no
matching bag, or with an empty detections CSV, are reported and skipped —
everything else is still processed.

Output is written to a new subfolder of <collected_dir>:
    marker_csv/<stem>.csv   (--format csv, default)
    marker_ply/<stem>.ply   (--format ply)

--format csv: one row per unique marker_id: marker_id,x,y,z, where x/y/z is
              the per-axis median over all of that marker's extracted points
              in this recording (robust to the rare stray bad detection).
--format ply: one point per CSV row (no aggregation), colored by marker_id,
              so you can see all detected instances at once and do your own
              filtering/manipulation afterward.

CSV columns expected in each detections CSV (header required):
    timestamp_ms,frame,marker_id,cx,cy,depth_m,c0x,c0y,c1x,c1y,c2x,c2y,c3x,c3y

Only frame, marker_id, cx, cy are used. depth_m from the CSV is ignored —
depth is re-sampled from the bag itself (median over a small pixel window),
since single-pixel reads at the marker center frequently land on holes.

Detections were made on the COLOR image, but the reconstructed .ply lives
in the DEPTH optical frame — so each point is deprojected using the color
intrinsics, then transformed through the color->depth extrinsics to land in
the same frame as the reconstructed point clouds.

Requires: pyrealsense2, numpy, open3d
    pip install pyrealsense2 numpy open3d
"""

import sys
import csv
import argparse
from pathlib import Path
from collections import defaultdict

import numpy as np
import pyrealsense2 as rs
import open3d as o3d


def load_detections(csv_path):
    by_frame = defaultdict(list)
    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            frame = int(float(row["frame"]))
            marker_id = int(float(row["marker_id"]))
            cx = float(row["cx"])
            cy = float(row["cy"])
            by_frame[frame].append((marker_id, cx, cy))
    return by_frame


def median_depth(depth_frame, u, v, win=2):
    """Median of valid depth readings in a (2*win+1)^2 window around (u,v)."""
    w, h = depth_frame.get_width(), depth_frame.get_height()
    vals = []
    for dy in range(-win, win + 1):
        for dx in range(-win, win + 1):
            x, y = int(round(u)) + dx, int(round(v)) + dy
            if 0 <= x < w and 0 <= y < h:
                d = depth_frame.get_distance(x, y)
                if d > 0:
                    vals.append(d)
    return float(np.median(vals)) if vals else None


# Distinct colors per marker_id, cycled if you have more markers than colors
PALETTE = np.array([
    [230, 25, 75], [60, 180, 75], [255, 225, 25], [0, 130, 200],
    [245, 130, 48], [145, 30, 180], [70, 240, 240], [240, 50, 230],
], dtype=np.float64) / 255.0


def extract_markers(bag_path, csv_path):
    """Returns (points, marker_ids, n_frames, n_skipped_depth) for one recording."""
    detections_by_frame = load_detections(csv_path)
    if not detections_by_frame:
        return [], [], 0, 0

    cfg = rs.config()
    cfg.enable_device_from_file(str(bag_path), repeat_playback=False)
    pipe = rs.pipeline()
    profile = pipe.start(cfg)
    profile.get_device().as_playback().set_real_time(False)

    depth_stream = profile.get_stream(rs.stream.depth).as_video_stream_profile()
    color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
    color_to_depth = color_stream.get_extrinsics_to(depth_stream)

    align = rs.align(rs.stream.color)

    points, marker_ids = [], []
    n_frames = 0
    n_skipped_depth = 0

    while True:
        ok, frameset = pipe.try_wait_for_frames(timeout_ms=2000)
        if not ok:
            break
        n_frames += 1

        frame_num = frameset.get_color_frame().get_frame_number()
        rows = detections_by_frame.get(frame_num)
        if not rows:
            continue

        aligned = align.process(frameset)
        depth_frame = aligned.get_depth_frame()
        color_frame = aligned.get_color_frame()
        color_intrin = color_frame.get_profile().as_video_stream_profile().get_intrinsics()

        for marker_id, cx, cy in rows:
            d = median_depth(depth_frame, cx, cy)
            if d is None:
                n_skipped_depth += 1
                continue

            p_color = rs.rs2_deproject_pixel_to_point(color_intrin, [cx, cy], d)
            p_depth = rs.rs2_transform_point_to_point(color_to_depth, p_color)
            # rs_marker's .ply is written via rs2::points::export_to_ply(), which
            # negates y and z (librealsense/src/points.cpp) before saving. Match
            # that here so marker points land in the same frame as those .ply files.
            p_depth = [p_depth[0], -p_depth[1], -p_depth[2]]

            points.append(p_depth)
            marker_ids.append(marker_id)

    pipe.stop()
    return points, marker_ids, n_frames, n_skipped_depth


def write_marker_ply(out_path, points, marker_ids):
    colors = [PALETTE[mid % len(PALETTE)] for mid in marker_ids]
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.array(points))
    pcd.colors = o3d.utility.Vector3dVector(np.array(colors))
    o3d.io.write_point_cloud(str(out_path), pcd)


def write_marker_csv(out_path, points, marker_ids):
    """One row per unique marker_id: marker_id,x,y,z (per-axis median of that marker's points)."""
    points = np.array(points)
    marker_ids = np.array(marker_ids)
    with open(out_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["marker_id", "x", "y", "z"])
        for mid in sorted(set(marker_ids)):
            xyz = np.median(points[marker_ids == mid], axis=0)
            writer.writerow([mid, *xyz])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("raw_dir", type=Path, help="folder of raw .bag recordings")
    ap.add_argument("collected_dir", type=Path,
                     help="collect_plys.py output folder (must contain detections/)")
    ap.add_argument("--format", choices=["ply", "csv"], default="csv",
                     help="output type: csv = one row per unique marker_id, "
                          "ply = one point per detection (default: csv)")
    args = ap.parse_args()

    detections_dir = args.collected_dir / "detections"
    if not detections_dir.is_dir():
        sys.exit(f"Error: {detections_dir} not found")

    csv_files = sorted(detections_dir.glob("*_detections.csv"))
    if not csv_files:
        sys.exit(f"No *_detections.csv files found in {detections_dir}")

    out_dir = args.collected_dir / ("marker_csv" if args.format == "csv" else "marker_ply")
    out_dir.mkdir(parents=True, exist_ok=True)

    n_ok = n_no_bag = n_empty = 0
    for csv_path in csv_files:
        stem = csv_path.name[: -len("_detections.csv")]
        bag_path = args.raw_dir / f"{stem}.bag"

        if not bag_path.is_file():
            print(f"{stem}: no matching bag in {args.raw_dir}, skipped")
            n_no_bag += 1
            continue

        points, marker_ids, n_frames, n_skipped_depth = extract_markers(bag_path, csv_path)
        if not points:
            print(f"{stem}: no marker points extracted (processed {n_frames} frames), skipped")
            n_empty += 1
            continue

        if args.format == "csv":
            out_path = out_dir / f"{stem}.csv"
            write_marker_csv(out_path, points, marker_ids)
            print(f"{stem}: {len(set(marker_ids))} unique markers -> {out_path.name}")
        else:
            out_path = out_dir / f"{stem}.ply"
            write_marker_ply(out_path, points, marker_ids)
            print(f"{stem}: {len(points)} points ({n_skipped_depth} skipped, no valid depth) -> {out_path.name}")

        n_ok += 1

    print(f"\nDone: {n_ok} written, {n_no_bag} missing bag, {n_empty} empty detections -> {out_dir}")


if __name__ == "__main__":
    main()
