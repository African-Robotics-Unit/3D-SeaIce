#!/usr/bin/env python3
"""
marker_points.py — extract 3D marker positions from a RealSense .bag +
AprilTag detection CSV, and save them as a small .ply point cloud (in the
same depth optical frame as your reconstructed point clouds) for overlay/QC
in CloudCompare, Open3D, etc.

Usage:
    python marker_points.py recording.bag detections.csv markers.ply

CSV columns expected (header required):
    timestamp_ms,frame,marker_id,cx,cy,depth_m,c0x,c0y,c1x,c1y,c2x,c2y,c3x,c3y

Only frame, marker_id, cx, cy are used. depth_m from the CSV is ignored —
depth is re-sampled from the bag itself (median over a small pixel window),
since single-pixel reads at the marker center frequently land on holes
(as in your sample row, where depth_m = 0.000).

Detections were made on the COLOR image, but your reconstructed .ply lives
in the DEPTH optical frame — so each point is deprojected using the color
intrinsics, then transformed through the color->depth extrinsics to land in
the same frame as your existing point clouds.

One output point per CSV row (no aggregation) — every detected marker
instance becomes a point, colored by marker_id, so you can see all of them
at once and do your own filtering/manipulation afterward.

Requires: pyrealsense2, numpy, open3d
    pip install pyrealsense2 numpy open3d
"""

import sys
import csv
import argparse
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


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag", help="input .bag recording")
    ap.add_argument("csv", help="AprilTag detections CSV")
    ap.add_argument("out_ply", help="output .ply of marker points")
    args = ap.parse_args()

    detections_by_frame = load_detections(args.csv)

    cfg = rs.config()
    cfg.enable_device_from_file(args.bag, repeat_playback=False)
    pipe = rs.pipeline()
    profile = pipe.start(cfg)
    profile.get_device().as_playback().set_real_time(False)

    depth_stream = profile.get_stream(rs.stream.depth).as_video_stream_profile()
    color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
    color_to_depth = color_stream.get_extrinsics_to(depth_stream)

    align = rs.align(rs.stream.color)

    points, colors = [], []
    n_frames = 0

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
                print(f"frame {frame_num} marker {marker_id}: no valid depth at ({cx:.0f},{cy:.0f}), skipped")
                continue

            p_color = rs.rs2_deproject_pixel_to_point(color_intrin, [cx, cy], d)
            p_depth = rs.rs2_transform_point_to_point(color_to_depth, p_color)

            points.append(p_depth)
            colors.append(PALETTE[marker_id % len(PALETTE)])

    pipe.stop()
    print(f"Processed {n_frames} frames, extracted {len(points)} marker points.")

    if not points:
        sys.exit("No marker points extracted — check that CSV 'frame' matches this bag's color frame_number().")

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.array(points))
    pcd.colors = o3d.utility.Vector3dVector(np.array(colors))
    o3d.io.write_point_cloud(args.out_ply, pcd)
    print(f"Wrote {args.out_ply}")


if __name__ == "__main__":
    main()