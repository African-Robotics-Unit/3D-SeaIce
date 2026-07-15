#!/usr/bin/env python3
"""
collect_plys.py — gather the per-recording outputs produced by rs_marker
(.bag recordings, .ply point clouds, AprilTag detections CSVs, IMU CSVs)
into flat, per-type output folders.

rs_marker writes each recording to its own timestamped subfolder:
    <session>/<YYYYMMDD_HHMMSS>/<YYYYMMDD_HHMMSS>.bag
    <session>/<YYYYMMDD_HHMMSS>/<YYYYMMDD_HHMMSS>.ply
    <session>/<YYYYMMDD_HHMMSS>/<YYYYMMDD_HHMMSS>_detections.csv
    <session>/<YYYYMMDD_HHMMSS>/imu.csv

Default mode — pass just a source folder (no dst):
    python3 tools/collect_plys.py /media/aru/Seagate/roughness/20260714

This is the common case: it collects ply/detections/imu into
"<src>_collected/<type>/" and the raw .bag files (flat, no subfolder) into
"<src>_raw/", both created next to src. Equivalent to running:
    python3 tools/collect_plys.py <src> <src>_collected --types ply detections imu
    python3 tools/collect_plys.py <src> <src>_raw        --types bag --flat

Custom mode — pass an explicit dst to control types/layout yourself:
    python3 tools/collect_plys.py <src> <out_dir> --types ply imu
    python3 tools/collect_plys.py <src> <out_dir> --mode symlink
    python3 tools/collect_plys.py <src> <out_dir> --types bag --flat

Custom mode walks src and, for each selected type, copies (or
symlinks/moves) every match into its own subfolder of dst:
    dst/bag/<YYYYMMDD_HHMMSS>.bag
    dst/ply/<YYYYMMDD_HHMMSS>.ply
    dst/detections/<YYYYMMDD_HHMMSS>_detections.csv
    dst/imu/<YYYYMMDD_HHMMSS>_imu.csv

.bag, .ply and detections filenames already embed the recording timestamp
so they are unique; imu.csv is named identically in every recording folder,
so it is always renamed to "<recording_folder>_imu.csv" to disambiguate.
The same rule kicks in automatically for the other types if a name
collision is ever found (or always, with --prefix-parent).

When collecting a single type, pass --flat to write straight into dst
instead of dst/<type>/ — handy for making a plain "just the .bag files"
folder (this is what default mode does for .bag under the hood).
"""

import argparse
import shutil
import sys
from collections import defaultdict
from pathlib import Path

# category -> (glob pattern relative to src, output subfolder name, always prefix with parent folder)
CATEGORIES = {
    "bag":        ("*.bag",              "bag",        False),
    "ply":        ("*.ply",              "ply",        False),
    "detections": ("*_detections.csv",   "detections", False),
    "imu":        ("imu.csv",            "imu",        True),
}


def parse_args():
    p = argparse.ArgumentParser(description="Collect rs_marker .bag / .ply / detections.csv / imu.csv files into flat per-type folders")
    p.add_argument("src", type=Path, help="Top-level folder containing rs_marker recordings")
    p.add_argument("dst", type=Path, nargs="?", default=None,
                   help="Output folder; a subfolder per type is created underneath it (unless --flat). "
                        "If omitted, default mode runs: ply/detections/imu into <src>_collected/, "
                        "bag into <src>_raw/ (flat) — see module docstring")
    p.add_argument("--types", nargs="+", choices=list(CATEGORIES), default=None,
                   help="Which file types to collect (default: all four; only valid with an explicit dst)")
    p.add_argument("--mode", choices=["copy", "symlink", "move"], default="copy",
                   help="How to place files in dst (default: copy)")
    p.add_argument("--prefix-parent", action="store_true",
                   help="Always prefix output filenames with '<parent_folder>_', "
                        "even for types that aren't normally ambiguous")
    p.add_argument("--flat", action="store_true",
                   help="Write files directly into dst instead of dst/<type>/. "
                        "Intended for collecting a single --types entry into a plain folder. "
                        "Only valid with an explicit dst.")
    return p.parse_args()


def collect(files, out_dir, mode, force_prefix, prefix_parent):
    by_basename = defaultdict(list)
    for f in files:
        by_basename[f.name].append(f)

    out_dir.mkdir(parents=True, exist_ok=True)

    placed = 0
    for f in files:
        needs_prefix = force_prefix or prefix_parent or len(by_basename[f.name]) > 1
        out_name = f"{f.parent.name}_{f.name}" if needs_prefix else f.name
        out_path = out_dir / out_name

        if out_path.exists():
            print(f"Skipping (already exists in dst): {out_path}", file=sys.stderr)
            continue

        if mode == "copy":
            shutil.copy2(f, out_path)
        elif mode == "symlink":
            out_path.symlink_to(f.resolve())
        elif mode == "move":
            shutil.move(str(f), str(out_path))

        placed += 1

    return placed


def run_collection(src, dst, types, mode, prefix_parent, flat):
    """Collect each of `types` from src into dst (or dst/<type>/ unless flat). Returns True if anything matched."""
    any_found = False
    for category in types:
        pattern, subdir, force_prefix = CATEGORIES[category]
        files = sorted(src.rglob(pattern))
        if not files:
            print(f"No files matching '{pattern}' found under {src} (type: {category})", file=sys.stderr)
            continue

        any_found = True
        out_dir = dst if flat else dst / subdir
        placed = collect(files, out_dir, mode, force_prefix, prefix_parent)
        print(f"{mode.capitalize()}d {placed}/{len(files)} {category} files into {out_dir}")

    return any_found


def main():
    args = parse_args()

    if not args.src.is_dir():
        sys.exit(f"Error: source folder not found: {args.src}")

    if args.dst is None:
        if args.types is not None or args.flat:
            sys.exit("Error: --types/--flat require an explicit dst folder; "
                      "omit them (and dst) to use default mode.")

        src_name = args.src.name or args.src.resolve().name  # handle trailing slash
        collected_dst = args.src.parent / f"{src_name}_collected"
        raw_dst       = args.src.parent / f"{src_name}_raw"
        print(f"Default mode (no dst given):\n"
              f"  {collected_dst}/  <- ply, detections, imu\n"
              f"  {raw_dst}/  <- bag (flat)\n")

        found_collected = run_collection(args.src, collected_dst, ["ply", "detections", "imu"],
                                          args.mode, args.prefix_parent, flat=False)
        found_raw = run_collection(args.src, raw_dst, ["bag"],
                                    args.mode, args.prefix_parent, flat=True)

        if not (found_collected or found_raw):
            sys.exit(f"No matching files found under {args.src}")
        return

    types = args.types if args.types is not None else list(CATEGORIES)
    if args.flat and len(types) > 1:
        print("Warning: --flat with multiple --types writes them all into the same "
              "folder — files from different types may collide.", file=sys.stderr)

    if not run_collection(args.src, args.dst, types, args.mode, args.prefix_parent, args.flat):
        sys.exit(f"No matching files found under {args.src} for types: {', '.join(types)}")


if __name__ == "__main__":
    main()
