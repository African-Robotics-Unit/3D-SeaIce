#!/usr/bin/env python3
"""
summarize_markers.py  —  analyse a detections CSV from rs_marker

Usage:
    python3 tools/summarize_markers.py detections.csv
    python3 tools/summarize_markers.py session_detections.csv --top 20
"""

import csv
import argparse
import sys
from collections import defaultdict

# ── CLI ───────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(description="Summarise AprilTag detection CSV")
    p.add_argument("csv", help="Path to detections CSV")
    p.add_argument("--top", type=int, default=10,
                   help="Number of best frames to show (default: 10)")
    p.add_argument("--timeline-width", type=int, default=60,
                   help="Width of ASCII timeline in characters (default: 60)")
    return p.parse_args()

# ── Parsing ───────────────────────────────────────────────────────────────────

def load(path):
    frames = defaultdict(dict)   # frame → {marker_id: row}
    rows   = []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            row["timestamp_ms"] = float(row["timestamp_ms"])
            row["frame"]        = int(row["frame"])
            row["marker_id"]    = int(row["marker_id"])
            row["depth_m"]      = float(row["depth_m"])
            frames[row["frame"]][row["marker_id"]] = row
            rows.append(row)
    return rows, frames

# ── Formatting helpers ────────────────────────────────────────────────────────

W = 44   # label column width

def hr():   print("─" * (W + 18))
def line(label, value): print(f"  {label:<{W}}{value}")

# ── Report sections ───────────────────────────────────────────────────────────

def section_overview(rows, frames, path):
    all_ids     = sorted({r["marker_id"] for r in rows})
    total_ms    = max(r["timestamp_ms"] for r in rows)
    n_frames_w  = len(frames)
    depths      = [r["depth_m"] for r in rows if r["depth_m"] > 0]

    print(f"\nAprilTag Detection Summary")
    print("=" * (W + 18))
    line("File",           path)
    line("Total detections",      len(rows))
    line("Frames with detections", n_frames_w)
    line("Duration (approx)",     f"{total_ms/1000:.1f}s")
    line("Marker IDs detected",   " ".join(str(i) for i in all_ids))
    if not depths:
        line("Depth readings", "⚠  all zero — depth map has holes at tag centres")
    else:
        line("Depth range", f"{min(depths):.3f}m – {max(depths):.3f}m")
    hr()

def section_per_marker(rows, frames):
    all_ids = sorted({r["marker_id"] for r in rows})
    n_total = len(frames)

    print("\n  Per-marker summary")
    hr()
    print(f"  {'ID':>4}  {'Frames':>8}  {'% frames':>9}  {'Depth avg':>10}  {'Depth range':>22}")
    hr()
    for mid in all_ids:
        det_frames = [f for f, m in frames.items() if mid in m]
        depths     = [frames[f][mid]["depth_m"] for f in det_frames
                      if frames[f][mid]["depth_m"] > 0]
        pct        = 100 * len(det_frames) / n_total if n_total else 0
        if depths:
            depth_str = f"{sum(depths)/len(depths):.3f}m"
            range_str = f"{min(depths):.3f} – {max(depths):.3f}m"
        else:
            depth_str = "—"
            range_str = "no valid readings"
        print(f"  {mid:>4}  {len(det_frames):>8}  {pct:>8.1f}%  {depth_str:>10}  {range_str:>22}")
    hr()

def section_marker_counts(frames):
    from collections import Counter
    count_dist = Counter(len(m) for m in frames.values())
    max_seen   = max(count_dist)

    print("\n  Simultaneous markers per frame")
    hr()
    print(f"  {'Markers visible':>18}  {'Frames':>8}  {'Bar'}")
    hr()
    total = sum(count_dist.values())
    for n in sorted(count_dist, reverse=True):
        bar = "█" * int(30 * count_dist[n] / total)
        print(f"  {n:>18}  {count_dist[n]:>8}  {bar}")
    hr()
    return max_seen

def section_best_frames(frames, max_seen, top_n):
    # Sort: most markers first, then earliest timestamp
    ranked = sorted(
        frames.items(),
        key=lambda kv: (-len(kv[1]), kv[1][next(iter(kv[1]))]["timestamp_ms"])
    )

    print(f"\n  Best {top_n} frames (most markers visible simultaneously)")
    hr()
    print(f"  {'Rank':>5}  {'Frame':>7}  {'Time(s)':>8}  {'Count':>6}  IDs")
    hr()
    for rank, (fnum, markers) in enumerate(ranked[:top_n], 1):
        ts   = next(iter(markers.values()))["timestamp_ms"] / 1000
        ids  = " ".join(str(i) for i in sorted(markers))
        star = " ★" if len(markers) == max_seen else ""
        print(f"  {rank:>5}  {fnum:>7}  {ts:>8.2f}s  {len(markers):>6}  {ids}{star}")
    hr()

    # Recommended frame
    best_frame, best_markers = ranked[0]
    best_ts = next(iter(best_markers.values()))["timestamp_ms"] / 1000
    print(f"\n  Recommended frame: {best_frame}  "
          f"(t={best_ts:.2f}s, {len(best_markers)} markers: "
          f"{' '.join(str(i) for i in sorted(best_markers))})")

def section_timeline(frames, width):
    if not frames:
        return
    all_frame_nums = sorted(frames)
    f_min, f_max   = all_frame_nums[0], all_frame_nums[-1]
    f_range        = max(f_max - f_min, 1)
    all_ids        = sorted({mid for m in frames.values() for mid in m})
    id_symbols     = {mid: str(i) for i, mid in enumerate(all_ids, 1)}

    # Bin frames into timeline slots
    bins = ["." * len(all_ids)] * width
    for fnum, markers in frames.items():
        slot = int((fnum - f_min) / f_range * (width - 1))
        chars = list(bins[slot])
        for i, mid in enumerate(all_ids):
            if mid in markers:
                chars[i] = id_symbols[mid]
        bins[slot] = "".join(chars)

    print(f"\n  Detection timeline  (each char = ~{f_range/width:.0f} frames)")
    print(f"  IDs: " + "  ".join(f"{id_symbols[m]}={m}" for m in all_ids))
    hr()
    print(f"  |{''.join(bins)}|")
    print(f"  frame {f_min:<{width//2-4}}frame {f_max}")
    hr()

# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    args = parse_args()
    try:
        rows, frames = load(args.csv)
    except FileNotFoundError:
        print(f"Error: file not found: {args.csv}", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"Error reading CSV: {e}", file=sys.stderr)
        sys.exit(1)

    if not rows:
        print("CSV contains no detections.")
        sys.exit(0)

    section_overview(rows, frames, args.csv)
    section_per_marker(rows, frames)
    section_marker_counts(frames)
    max_seen = max(len(m) for m in frames.values())
    section_best_frames(frames, max_seen, args.top)
    section_timeline(frames, args.timeline_width)
    print()

if __name__ == "__main__":
    main()
