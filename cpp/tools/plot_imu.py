#!/usr/bin/env python3
"""
plot_imu.py — extract IMU (accelerometer + gyroscope) data from RealSense
.bag recordings: one PNG figure per bag plotting it over time, and one CSV
per bag with the raw sample stream.

Usage:
    python3 plot_imu.py <bag_dir> [--graphs-dir DIR] [--csv-dir DIR]

<bag_dir>      folder of raw .bag recordings, e.g. .../rawTest
--graphs-dir   where to write <stem>_imu.png (default: <bag_dir>/imu_graphs)
--csv-dir      where to write <stem>_imu.csv (default: <bag_dir>/imu_total)

CSV columns: timestamp_ms,stream,x,y,z — one row per raw motion sample
(stream is "accel" or "gyro"; x/y/z in m/s² for accel, rad/s for gyro),
sorted by timestamp_ms as recorded on the device.

Requires: pyrealsense2, numpy, matplotlib
    pip install pyrealsense2 numpy matplotlib
"""

import argparse
import csv
import sys
from pathlib import Path

import numpy as np
import pyrealsense2 as rs
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# Chart chrome (light mode)
SURFACE = "#fcfcfb"
INK_PRIMARY = "#0b0b0b"
INK_SECONDARY = "#52514e"
INK_MUTED = "#898781"
GRIDLINE = "#e1e0d9"
BASELINE = "#c3c2b7"

# Categorical palette, fixed order: x, y, z
AXIS_COLORS = {"x": "#2a78d6", "y": "#1baf7a", "z": "#eda100"}

PANEL_SPECS = [
    ("accel", "Accelerometer", "m/s²"),
    ("gyro", "Gyroscope", "rad/s"),
]


def extract_imu(bag_path):
    """Returns {'accel': (ts_ms, xyz) or None, 'gyro': ...}, ts_ms = raw absolute device timestamps, sorted."""
    cfg = rs.config()
    cfg.enable_device_from_file(str(bag_path), repeat_playback=False)
    pipe = rs.pipeline()
    profile = pipe.start(cfg)
    profile.get_device().as_playback().set_real_time(False)

    samples = {"accel": [], "gyro": []}

    while True:
        ok, frameset = pipe.try_wait_for_frames(timeout_ms=2000)
        if not ok:
            break
        for frame in frameset:
            if not frame.is_motion_frame():
                continue
            mf = frame.as_motion_frame()
            stream = mf.get_profile().stream_type()
            data = mf.get_motion_data()
            ts_ms = mf.get_timestamp()
            if stream == rs.stream.accel:
                samples["accel"].append((ts_ms, data.x, data.y, data.z))
            elif stream == rs.stream.gyro:
                samples["gyro"].append((ts_ms, data.x, data.y, data.z))

    pipe.stop()

    out = {}
    for key, rows in samples.items():
        if not rows:
            out[key] = None
            continue
        arr = np.array(sorted(rows))
        out[key] = (arr[:, 0], arr[:, 1:4])
    return out


def plot_imu(stem, imu, out_path):
    panels = [(key, title, unit) for key, title, unit in PANEL_SPECS if imu[key] is not None]

    fig, axes = plt.subplots(len(panels), 1, figsize=(10, 3.2 * len(panels)),
                              sharex=True, facecolor=SURFACE)
    if len(panels) == 1:
        axes = [axes]

    for ax, (key, title, unit) in zip(axes, panels):
        ts_ms, xyz = imu[key]
        t = (ts_ms - ts_ms[0]) / 1000.0
        ax.set_facecolor(SURFACE)
        for i, axis in enumerate("xyz"):
            ax.plot(t, xyz[:, i], label=axis, color=AXIS_COLORS[axis], linewidth=1.5)
        ax.set_title(title, color=INK_PRIMARY, fontsize=11, loc="left")
        ax.set_ylabel(unit, color=INK_SECONDARY, fontsize=9)
        ax.tick_params(colors=INK_MUTED, labelsize=8)
        ax.grid(True, color=GRIDLINE, linewidth=0.8)
        for spine in ax.spines.values():
            spine.set_color(BASELINE)
        ax.legend(loc="upper right", frameon=False, labelcolor=INK_SECONDARY, fontsize=8)

    axes[-1].set_xlabel("Time (s)", color=INK_SECONDARY, fontsize=9)
    fig.suptitle(stem, color=INK_PRIMARY, fontsize=12, y=0.995)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(out_path, dpi=150, facecolor=SURFACE)
    plt.close(fig)


def write_imu_csv(imu, out_path):
    """One row per raw motion sample across both streams, sorted by timestamp_ms."""
    rows = []
    for stream in ("accel", "gyro"):
        if imu[stream] is None:
            continue
        ts_ms, xyz = imu[stream]
        for t, (x, y, z) in zip(ts_ms, xyz):
            rows.append((t, stream, x, y, z))
    rows.sort(key=lambda r: r[0])

    with open(out_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp_ms", "stream", "x", "y", "z"])
        for t, stream, x, y, z in rows:
            writer.writerow([f"{t:.3f}", stream, f"{x:.6f}", f"{y:.6f}", f"{z:.6f}"])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag_dir", type=Path, help="folder of raw .bag recordings")
    ap.add_argument("--graphs-dir", type=Path, default=None,
                     help="where to write <stem>_imu.png (default: <bag_dir>/imu_graphs)")
    ap.add_argument("--csv-dir", type=Path, default=None,
                     help="where to write <stem>_imu.csv (default: <bag_dir>/imu_total)")
    args = ap.parse_args()

    if not args.bag_dir.is_dir():
        sys.exit(f"Error: {args.bag_dir} not found")

    bag_files = sorted(args.bag_dir.glob("*.bag"))
    if not bag_files:
        sys.exit(f"No .bag files found in {args.bag_dir}")

    graphs_dir = args.graphs_dir or (args.bag_dir / "imu_graphs")
    csv_dir = args.csv_dir or (args.bag_dir / "imu_total")
    graphs_dir.mkdir(parents=True, exist_ok=True)
    csv_dir.mkdir(parents=True, exist_ok=True)

    n_ok = n_empty = 0
    for bag_path in bag_files:
        stem = bag_path.stem
        imu = extract_imu(bag_path)
        if all(v is None for v in imu.values()):
            print(f"{stem}: no IMU data found, skipped")
            n_empty += 1
            continue

        png_path = graphs_dir / f"{stem}_imu.png"
        csv_path = csv_dir / f"{stem}_imu.csv"
        plot_imu(stem, imu, png_path)
        write_imu_csv(imu, csv_path)

        counts = ", ".join(f"{k}={len(v[0])} samples" for k, v in imu.items() if v is not None)
        print(f"{stem}: {counts} -> {png_path.name}, {csv_path.name}")
        n_ok += 1

    print(f"\nDone: {n_ok} written, {n_empty} with no IMU data -> {graphs_dir}, {csv_dir}")


if __name__ == "__main__":
    main()
