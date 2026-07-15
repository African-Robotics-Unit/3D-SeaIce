#!/usr/bin/env python3
"""
plot_gyro_outliers.py — flag and plot gyroscope outliers across a whole
dataset of IMU CSVs written by plot_imu.py.

Usage:
    python3 plot_gyro_outliers.py <csv_dir> [--out DIR] [--threshold Z]

<csv_dir>     folder of <stem>_imu.csv files written by plot_imu.py
              (i.e. the imu_total/ folder)
--out         where to write gyro_outliers.png (default: <csv_dir>/../imu_outliers)
--threshold   modified z-score above which a sample is flagged an outlier
              (default: 3.5, the standard Iglewicz & Hoaglin cutoff)

Method: pools the gyroscope angular-rate magnitude (sqrt(x²+y²+z²)) from
every recording, then flags outliers via the modified z-score
0.6745 * (x - median) / MAD — robust to the fact that most samples are
near-stationary noise, so a handful of real spikes don't skew the
threshold the way a mean/stdev-based z-score would.

Requires: numpy, matplotlib
    pip install numpy matplotlib
"""

import argparse
import csv
import sys
from pathlib import Path

import numpy as np
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

INLIER_COLOR = "#898781"
SERIES_COLOR = "#2a78d6"      # categorical slot 1 — bar chart
OUTLIER_COLOR = "#d03b3b"     # status: critical — reserved for anomalies


def load_gyro_magnitudes(csv_dir):
    """Returns list of (stem, timestamp_ms, magnitude), pooled across all *_imu.csv files, in file/row order."""
    samples = []
    for csv_path in sorted(csv_dir.glob("*_imu.csv")):
        stem = csv_path.name[: -len("_imu.csv")]
        with open(csv_path, newline="") as f:
            reader = csv.DictReader(f)
            for row in reader:
                if row["stream"] != "gyro":
                    continue
                x, y, z = float(row["x"]), float(row["y"]), float(row["z"])
                mag = (x * x + y * y + z * z) ** 0.5
                samples.append((stem, float(row["timestamp_ms"]), mag))
    return samples


def flag_outliers(magnitudes, threshold):
    """Modified z-score (median + MAD). Returns (modified_z, is_outlier) arrays."""
    median = np.median(magnitudes)
    mad = np.median(np.abs(magnitudes - median))
    if mad == 0:
        z = np.zeros_like(magnitudes)
    else:
        z = 0.6745 * (magnitudes - median) / mad
    return z, np.abs(z) > threshold


def plot_outliers(samples, is_outlier, threshold, out_path):
    stems = [s[0] for s in samples]
    magnitudes = np.array([s[2] for s in samples])
    unique_stems = sorted(set(stems), key=stems.index)

    # Boundaries between recordings, for the vertical separators and x-tick placement
    counts = [stems.count(stem) for stem in unique_stems]
    boundaries = np.cumsum([0] + counts)
    centers = (boundaries[:-1] + boundaries[1:]) / 2
    index = np.arange(len(samples))

    outlier_counts = [int(is_outlier[boundaries[i]:boundaries[i + 1]].sum()) for i in range(len(unique_stems))]

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(max(10, 0.4 * len(unique_stems)), 8),
                                    facecolor=SURFACE, gridspec_kw={"height_ratios": [2, 1]})

    # Panel 1: magnitude scatter across the whole dataset, outliers highlighted
    ax1.set_facecolor(SURFACE)
    for b in boundaries[1:-1]:
        ax1.axvline(b, color=GRIDLINE, linewidth=0.8)
    ax1.scatter(index[~is_outlier], magnitudes[~is_outlier], s=6, color=INLIER_COLOR,
                label="inlier", zorder=2)
    ax1.scatter(index[is_outlier], magnitudes[is_outlier], s=22, color=OUTLIER_COLOR,
                label=f"outlier (|z| > {threshold})", zorder=3)
    ax1.set_ylabel("Gyro |ω| (rad/s)", color=INK_SECONDARY, fontsize=9)
    ax1.set_title("Gyroscope magnitude — outliers across dataset", color=INK_PRIMARY, fontsize=11, loc="left")
    ax1.tick_params(colors=INK_MUTED, labelsize=8)
    ax1.set_xticks(centers)
    ax1.set_xticklabels(unique_stems, rotation=90, fontsize=6, color=INK_MUTED)
    ax1.grid(True, axis="y", color=GRIDLINE, linewidth=0.8)
    for spine in ax1.spines.values():
        spine.set_color(BASELINE)
    ax1.legend(loc="upper right", frameon=False, labelcolor=INK_SECONDARY, fontsize=8)

    # Panel 2: outlier count per recording
    ax2.set_facecolor(SURFACE)
    ax2.bar(range(len(unique_stems)), outlier_counts, color=SERIES_COLOR, width=0.7)
    ax2.set_ylabel("Outlier count", color=INK_SECONDARY, fontsize=9)
    ax2.set_xticks(range(len(unique_stems)))
    ax2.set_xticklabels(unique_stems, rotation=90, fontsize=6, color=INK_MUTED)
    ax2.tick_params(colors=INK_MUTED, labelsize=8)
    ax2.grid(True, axis="y", color=GRIDLINE, linewidth=0.8)
    for spine in ax2.spines.values():
        spine.set_color(BASELINE)

    fig.tight_layout()
    fig.savefig(out_path, dpi=150, facecolor=SURFACE)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv_dir", type=Path, help="folder of <stem>_imu.csv files (imu_total/)")
    ap.add_argument("--out", type=Path, default=None,
                     help="where to write gyro_outliers.png (default: <csv_dir>/../imu_outliers)")
    ap.add_argument("--threshold", type=float, default=3.5,
                     help="modified z-score cutoff for flagging an outlier (default: 3.5)")
    args = ap.parse_args()

    if not args.csv_dir.is_dir():
        sys.exit(f"Error: {args.csv_dir} not found")

    samples = load_gyro_magnitudes(args.csv_dir)
    if not samples:
        sys.exit(f"No gyro samples found in *_imu.csv files under {args.csv_dir}")

    magnitudes = np.array([s[2] for s in samples])
    z, is_outlier = flag_outliers(magnitudes, args.threshold)

    out_dir = args.out or (args.csv_dir.parent / "imu_outliers")
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / "gyro_outliers.png"
    plot_outliers(samples, is_outlier, args.threshold, out_path)

    n_outliers = int(is_outlier.sum())
    print(f"{len(samples)} gyro samples, {n_outliers} outliers (threshold |z| > {args.threshold})")
    if n_outliers:
        print("\nTop outliers:")
        order = np.argsort(-np.abs(z))[:min(10, n_outliers)]
        for i in order:
            stem, ts_ms, mag = samples[i]
            print(f"  {stem}  t={ts_ms:.1f}ms  |ω|={mag:.4f} rad/s  z={z[i]:.1f}")

    print(f"\nWrote {out_path}")


if __name__ == "__main__":
    main()
