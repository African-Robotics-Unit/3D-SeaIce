"""
rs_depth_filter_pipeline.py
============================
Applies the Intel RealSense SDK post-processing filter chain to raw depth
images extracted from ROS2 bags, without requiring a live camera or SDK bag
format. Uses pyrealsense2 software_device to inject numpy depth arrays into
the true SDK filter chain (identical to RealSense Viewer output).

Pipeline stages:
  [1] EXTRACT   -- Read raw depth frames and camera intrinsics from ROS2 bag
  [2] CLEAN     -- Zero out sentinel values and out-of-range pixels
  [3] INJECT    -- Wrap numpy array in pyrealsense2 software_device frame
  [4] FILTER    -- Apply SDK filter chain (Decimation > Spatial > Temporal > HoleFill)
  [5] REPROJECT -- Convert filtered depth back to (N,3) xyz point cloud
  [6] COMPARE   -- Visualise and compute plane-fit noise metrics

Requirements:
  pip install pyrealsense2 rosbags open3d numpy

Author: ARU/MARIS
"""

# ─────────────────────────────────────────────────────────────────────────────
# IMPORTS
# ─────────────────────────────────────────────────────────────────────────────

import numpy as np
import open3d as o3d
import pyrealsense2 as rs
from pathlib import Path
from rosbags.highlevel import AnyReader

# ─────────────────────────────────────────────────────────────────────────────
# CONFIGURATION
# Edit these values before running.
# ─────────────────────────────────────────────────────────────────────────────

CONFIG = {
    # Path to one rosbag2 directory (folder starting with 'rosbag2_...')
    "bagdir":     r"C:\Users\agori\Documents\MATLAB\MSc\bigfiles\testROS2\rosbag2_2022_12_07-12_17_52",

    # Which RealSense frame to use (matches rs_frame_idx in ros2_get_raw())
    "frame_idx":  3,

    # Filter preset: "none" | "default" | "high_accuracy" | "high_density"
    "preset":     "high_density",

    # Depth clamp range in mm -- zero out pixels outside this window.
    # Set max_mm to just above the furthest genuine ice surface return.
    # Confirm from spatial distribution plot before batch processing.
    "min_mm":     500,
    "max_mm":     6000,

    # ROS2 topic names (adjust if your bag uses different names)
    "depth_topic": "/camera/depth/image_rect_raw",
    "info_topic":  "/camera/depth/camera_info",
}

# ─────────────────────────────────────────────────────────────────────────────
# PRESET DEFINITIONS
# (decimation_mag, spatial_mag, spatial_alpha, spatial_delta,
#  temporal_alpha, temporal_delta, hole_fill_mode)
# ─────────────────────────────────────────────────────────────────────────────

PRESETS = {
    "none":          None,
    "default":       (2, 2, 0.50, 20, 0.40, 20, 0),
    "high_accuracy": (2, 2, 0.50, 20, 0.40, 20, 1),
    "high_density":  (2, 2, 0.50, 20, 0.40, 20, 2),
}

# ─────────────────────────────────────────────────────────────────────────────
# [1] EXTRACT
# ─────────────────────────────────────────────────────────────────────────────

def extract_camera_info(bagdir, info_topic="/camera/depth/camera_info"):
    """
    Read the first CameraInfo message from the bag and return an
    intrinsics dict with keys: width, height, fx, fy, ppx, ppy, d.
    """
    with AnyReader([Path(bagdir)]) as reader:
        for conn, ts, raw in reader.messages():
            if conn.topic == info_topic:
                msg = reader.deserialize(raw, conn.msgtype)
                return {
                    "width":  msg.width,
                    "height": msg.height,
                    "fx":     msg.k[0],   # K = [fx, 0, ppx, 0, fy, ppy, 0, 0, 1]
                    "fy":     msg.k[4],
                    "ppx":    msg.k[2],
                    "ppy":    msg.k[5],
                    "d":      list(msg.d)[:5],
                }
    raise RuntimeError(f"No CameraInfo found on topic: {info_topic}")


def extract_depth_frames(bagdir, depth_topic="/camera/depth/image_rect_raw"):
    """
    Extract all depth frames from a ROS2 bag as uint16 numpy arrays.

    Returns a list of dicts with keys:
      ts       -- ROS2 timestamp (nanoseconds)
      depth_mm -- (H, W) uint16 array, values in millimetres
    """
    frames = []
    with AnyReader([Path(bagdir)]) as reader:
        for conn, ts, raw in reader.messages():
            if conn.topic == depth_topic:
                msg = reader.deserialize(raw, conn.msgtype)
                assert msg.encoding == "16UC1", \
                    f"Unexpected depth encoding: {msg.encoding}"
                depth_mm = np.frombuffer(
                    msg.data, dtype=np.uint16
                ).reshape(msg.height, msg.width).copy()
                frames.append({"ts": ts, "depth_mm": depth_mm})
    return frames

# ─────────────────────────────────────────────────────────────────────────────
# [2] CLEAN
# ─────────────────────────────────────────────────────────────────────────────

def clamp_depth(depth_mm, min_mm=500, max_mm=6000):
    """
    Zero out pixels that are:
      - The 65535 sentinel (RealSense out-of-range flag)
      - Below min_mm (rig / deck returns)
      - Above max_mm (background / sky returns)

    The SDK treats zero as invalid. Do NOT use NaN here.
    Adjust min_mm and max_mm to match your scanning geometry.
    """
    out = depth_mm.copy()
    out[depth_mm == 65535]  = 0
    out[depth_mm < min_mm]  = 0
    out[depth_mm > max_mm]  = 0
    return out

# ─────────────────────────────────────────────────────────────────────────────
# [3] INJECT
# ─────────────────────────────────────────────────────────────────────────────

def numpy_to_rs_depth_frame(depth_mm, intrinsics, frame_number=0):
    """
    Inject a uint16 numpy depth array into a pyrealsense2 software_device
    and return an rs.depth_frame suitable for the SDK filter chain.

    No live camera is required. The software_device acts as a fake camera.

    Key fixes applied:
      - fh.profile must be set to a video_stream_profile (not base stream_profile)
        obtained via as_video_stream_profile() -- without this, on_video_frame hangs.
      - Validity checked via bool(frame) not frame.is_valid() (removed in SDK 2.50+).

    Parameters
    ----------
    depth_mm     : np.ndarray, shape (H, W), dtype uint16, values in mm
    intrinsics   : dict from extract_camera_info()
    frame_number : int, frame index (used for temporal filter state tracking)

    Returns
    -------
    rs.depth_frame
    """
    H, W = depth_mm.shape
    assert depth_mm.dtype == np.uint16, \
        f"depth_mm must be uint16, got {depth_mm.dtype}"

    # Create software device and depth sensor
    dev    = rs.software_device()
    sensor = dev.add_sensor("depth")

    # Define the depth stream
    stream        = rs.video_stream()
    stream.type   = rs.stream.depth
    stream.index  = 0
    stream.uid    = 0
    stream.fps    = 30
    stream.bpp    = 2               # bytes per pixel (uint16 = 2)
    stream.fmt    = rs.format.z16
    stream.width  = W
    stream.height = H
    sensor.add_video_stream(stream)
    sensor.add_read_only_option(rs.option.depth_units, 0.001)  # mm -> metres

    # Open and start the sensor
    syncer = rs.syncer()
    sensor.open(sensor.get_stream_profiles()[0])
    sensor.start(syncer)

    # Get stream profile as video_stream_profile -- required for frame routing.
    # Must call as_video_stream_profile() -- base stream_profile type causes
    # a TypeError on fh.profile assignment.
    depth_profile = sensor.get_stream_profiles()[0].as_video_stream_profile()

    # Build the software frame
    fh              = rs.software_video_frame()
    fh.pixels       = depth_mm.tobytes()
    fh.stride       = W * 2         # bytes per row
    fh.bpp          = 2
    fh.frame_number = frame_number
    fh.timestamp    = frame_number * (1000.0 / 30.0)   # ms
    fh.domain       = rs.timestamp_domain.hardware_clock
    fh.profile      = depth_profile  # CRITICAL: without this on_video_frame hangs

    # Inject frame and retrieve it through the syncer
    sensor.on_video_frame(fh)
    frameset    = syncer.wait_for_frames(5000)
    depth_frame = frameset.get_depth_frame()

    sensor.stop()
    sensor.close()

    return depth_frame

# ─────────────────────────────────────────────────────────────────────────────
# [4] FILTER
# ─────────────────────────────────────────────────────────────────────────────

def build_filter_chain(preset="high_density"):
    """
    Build the RealSense Viewer post-processing filter chain.

    Filter order matches Intel's recommended pipeline:
      Decimation > Depth2Disparity > Spatial > Temporal > Disparity2Depth > HoleFill

    The chain is STATEFUL -- the temporal filter accumulates history across
    calls. Keep the returned list alive across all frames in one bag.
    Reset (call build_filter_chain again) between bags.

    Parameters
    ----------
    preset : str, one of PRESETS keys

    Returns
    -------
    list of rs filter objects, or empty list for preset="none"
    """
    if PRESETS[preset] is None:
        return []

    dec_mag, spat_mag, spat_alpha, spat_delta, \
        temp_alpha, temp_delta, hole_fill = PRESETS[preset]

    dec = rs.decimation_filter()
    dec.set_option(rs.option.filter_magnitude, dec_mag)

    d2d = rs.disparity_transform(True)    # depth -> disparity

    spat = rs.spatial_filter()
    spat.set_option(rs.option.filter_magnitude,    float(spat_mag))
    spat.set_option(rs.option.filter_smooth_alpha,  spat_alpha)
    spat.set_option(rs.option.filter_smooth_delta,  float(spat_delta))

    temp = rs.temporal_filter()
    temp.set_option(rs.option.filter_smooth_alpha,  temp_alpha)
    temp.set_option(rs.option.filter_smooth_delta,  float(temp_delta))

    d2d_inv = rs.disparity_transform(False)  # disparity -> depth

    hole = rs.hole_filling_filter()
    hole.set_option(rs.option.holes_fill, hole_fill)

    return [dec, d2d, spat, temp, d2d_inv, hole]


def apply_filter_chain(rs_depth_frame, filter_chain):
    """
    Apply a list of rs filters sequentially to an rs.depth_frame.

    Parameters
    ----------
    rs_depth_frame : rs.depth_frame from numpy_to_rs_depth_frame()
    filter_chain   : list from build_filter_chain()

    Returns
    -------
    rs.frame (filtered depth)
    """
    frame = rs_depth_frame
    for f in filter_chain:
        frame = f.process(frame)
    return frame

# ─────────────────────────────────────────────────────────────────────────────
# [5] REPROJECT
# ─────────────────────────────────────────────────────────────────────────────

def depth_frame_to_xyz(depth_mm_arr, intrinsics, dec_mag=1):
    """
    Convert a 2D uint16 depth array to an (N, 3) xyz point cloud in metres
    using the pinhole camera model:
        X = (u - ppx) * Z / fx
        Y = (v - ppy) * Z / fy
        Z = depth_mm * 0.001

    If decimation was applied, intrinsics are scaled down by dec_mag.
    Zero pixels (invalid after filtering) are excluded.

    Parameters
    ----------
    depth_mm_arr : np.ndarray uint16 (H, W)
    intrinsics   : dict from extract_camera_info()
    dec_mag      : int, decimation magnitude applied (1 = no decimation)

    Returns
    -------
    np.ndarray float32 (N, 3), xyz in metres
    """
    scale = float(dec_mag)
    fx  = intrinsics["fx"]  / scale
    fy  = intrinsics["fy"]  / scale
    ppx = intrinsics["ppx"] / scale
    ppy = intrinsics["ppy"] / scale

    H, W = depth_mm_arr.shape
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    Z    = depth_mm_arr.astype(np.float32) * 0.001   # mm -> metres

    valid = depth_mm_arr > 0
    X = (u[valid] - ppx) * Z[valid] / fx
    Y = (v[valid] - ppy) * Z[valid] / fy

    return np.stack([X, Y, Z[valid]], axis=1)

# ─────────────────────────────────────────────────────────────────────────────
# [6] COMPARE
# ─────────────────────────────────────────────────────────────────────────────

def compare_clouds(xyz_raw, xyz_filtered):
    """
    Open3D side-by-side visualisation.
    Raw cloud in red, filtered cloud in green.
    """
    pcd_raw = o3d.geometry.PointCloud()
    pcd_raw.points = o3d.utility.Vector3dVector(xyz_raw)
    pcd_raw.paint_uniform_color([1, 0, 0])

    pcd_filt = o3d.geometry.PointCloud()
    pcd_filt.points = o3d.utility.Vector3dVector(xyz_filtered)
    pcd_filt.paint_uniform_color([0, 0.7, 0])

    print(f"  Raw:      {len(xyz_raw):,} points")
    print(f"  Filtered: {len(xyz_filtered):,} points  "
          f"({100*len(xyz_filtered)/max(len(xyz_raw),1):.1f}% of raw)")

    o3d.visualization.draw_geometries(
        [pcd_raw, pcd_filt],
        window_name="Raw (red) vs Filtered (green)",
    )


def plane_fit_metric(xyz, distance_threshold=0.005, silent=False):
    """
    RANSAC plane fit on a point cloud.
    Reports mean and std of per-point residuals from the fitted plane in mm.

    Parameters
    ----------
    xyz                : np.ndarray (N, 3)
    distance_threshold : float, RANSAC inlier threshold in metres (default 5 mm)
    silent             : bool, suppress print output for batch use

    Returns
    -------
    residuals : np.ndarray (N,), per-point distances from plane in metres
    """
    if len(xyz) < 10:
        if not silent:
            print("  Too few points for plane fit.")
        return np.array([np.nan])

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)
    plane, inliers = pcd.segment_plane(
        distance_threshold, ransac_n=3, num_iterations=1000
    )
    a, b, c, d = plane
    normal    = np.array([a, b, c])
    residuals = np.abs(xyz @ normal + d) / np.linalg.norm(normal)

    if not silent:
        print(f"  Points:        {len(xyz):,}")
        print(f"  Inliers:       {len(inliers):,}  "
              f"({100*len(inliers)/len(xyz):.1f}%)")
        print(f"  Mean residual: {residuals.mean()*1000:.2f} mm")
        print(f"  Std  residual: {residuals.std()*1000:.2f} mm")

    return residuals

# ─────────────────────────────────────────────────────────────────────────────
# SINGLE BAG: run the full pipeline on one bag, one frame
# ─────────────────────────────────────────────────────────────────────────────
def run_single_bag(config):
    bagdir      = config["bagdir"]
    frame_idx   = config["frame_idx"]
    preset      = config["preset"]
    min_mm      = config["min_mm"]
    max_mm      = config["max_mm"]
    depth_topic = config.get("depth_topic", "/camera/depth/image_rect_raw")
    info_topic  = config.get("info_topic",  "/camera/depth/camera_info")

    print(f"Bag:    {bagdir}")
    print(f"Preset: {preset}")
    print(f"Clamp:  {min_mm} -- {max_mm} mm")

    # [1] Extract
    print("\n[1] Extracting...")
    intrinsics   = extract_camera_info(bagdir, info_topic)
    depth_frames = extract_depth_frames(bagdir, depth_topic)
    print(f"  Intrinsics: {intrinsics['width']}x{intrinsics['height']}  "
          f"fx={intrinsics['fx']:.1f}  fy={intrinsics['fy']:.1f}")
    print(f"  Frames extracted: {len(depth_frames)}")

    if frame_idx >= len(depth_frames):
        raise IndexError(
            f"frame_idx={frame_idx} but only {len(depth_frames)} frames in bag."
        )

    # [2] Clean
    raw_clamped = clamp_depth(depth_frames[frame_idx]["depth_mm"], min_mm, max_mm)
    print(f"\n[2] After clamp (frame {frame_idx}): "
          f"valid={(raw_clamped > 0).sum():,}  "
          f"zeros={(raw_clamped == 0).sum():,}")

    # [3+4] Inject and filter
    print(f"\n[3+4] Injecting and filtering (frames 0 to {frame_idx})...")
    filter_chain = build_filter_chain(preset)
    dec_mag      = PRESETS[preset][0] if PRESETS[preset] else 1

    for i in range(frame_idx + 1):
        clamped        = clamp_depth(depth_frames[i]["depth_mm"], min_mm, max_mm)
        rs_frame       = numpy_to_rs_depth_frame(clamped, intrinsics, frame_number=i)

        if not bool(rs_frame):
            raise RuntimeError(f"Frame {i}: rs_frame invalid after injection.")

        filtered_frame = apply_filter_chain(rs_frame, filter_chain)

        # Extract array to inspect shape -- avoids get_width() on base frame type
        filtered_arr = np.asanyarray(filtered_frame.get_data())
        print(f"  Frame {i}: OK  shape={filtered_arr.shape}  "
              f"valid={(filtered_arr > 0).sum():,}")

    # filtered_arr from last iteration is the target frame
    # [5] Reproject
    print("\n[5] Reprojecting...")
    xyz_raw      = depth_frame_to_xyz(raw_clamped,  intrinsics, dec_mag=1)
    xyz_filtered = depth_frame_to_xyz(filtered_arr, intrinsics, dec_mag=dec_mag)
    print(f"  Raw:      {len(xyz_raw):,} points")
    print(f"  Filtered: {len(xyz_filtered):,} points  "
          f"({100*len(xyz_filtered)/max(len(xyz_raw),1):.1f}% of raw)")

    # [6] Compare
    print("\n[6] Plane fit metrics:")
    print("  --- Raw ---")
    plane_fit_metric(xyz_raw)
    print("  --- Filtered ---")
    plane_fit_metric(xyz_filtered)

    print("\nLaunching Open3D visualisation (close window to exit)...")
    compare_clouds(xyz_raw, xyz_filtered)
    # [7] Export PLY
    print("\n[7] Exporting PLY files...")
    out_dir = Path(bagdir).parent / "ply_output" / preset
    out_dir.mkdir(parents=True, exist_ok=True)
    bag_stem = Path(bagdir).name

    def z_to_rgb_jet(z_values):
        z = z_values.astype(np.float32)
        z_min, z_max = np.nanmin(z), np.nanmax(z)
        if z_max == z_min:
            return np.full((len(z), 3), 128, dtype=np.uint8)
        t = (z - z_min) / (z_max - z_min)
        r = np.clip(1.5 - np.abs(4.0 * t - 3.0), 0, 1)
        g = np.clip(1.5 - np.abs(4.0 * t - 2.0), 0, 1)
        b = np.clip(1.5 - np.abs(4.0 * t - 1.0), 0, 1)
        return (np.stack([r, g, b], axis=1) * 255).astype(np.uint8)

    def save_ply(xyz, path, label=""):
        xyz  = xyz.astype(np.float32)
        rgb  = z_to_rgb_jet(xyz[:, 2])
        N    = len(xyz)
        path = Path(path)
        comment = "pointcloud saved from Realsense Viewer"
        if label:
            comment += f" [{label}]"
        header = "\n".join([
            "ply",
            "format binary_little_endian 1.0",
            f"comment {comment}",
            f"element vertex {N}",
            "property float x",
            "property float y",
            "property float z",
            "property uchar red",
            "property uchar green",
            "property uchar blue",
            "end_header",
            "",
        ]).encode("ascii")
        dtype = np.dtype([
            ("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
            ("red", "u1"), ("green", "u1"), ("blue", "u1"),
        ])
        data          = np.empty(N, dtype=dtype)
        data["x"]     = xyz[:, 0]
        data["y"]     = xyz[:, 1]
        data["z"]     = xyz[:, 2]
        data["red"]   = rgb[:, 0]
        data["green"] = rgb[:, 1]
        data["blue"]  = rgb[:, 2]
        with open(path, "wb") as f:
            f.write(header)
            f.write(data.tobytes())
        print(f"  Saved: {path.name}  "
              f"({N:,} pts, {path.stat().st_size/1e6:.1f} MB)")

    save_ply(
        xyz_raw,
        out_dir / f"{bag_stem}_frame{frame_idx}_raw.ply",
        label="raw",
    )
    save_ply(
        xyz_filtered,
        out_dir / f"{bag_stem}_frame{frame_idx}_{preset}_filtered.ply",
        label=f"{preset} dec={dec_mag}",
    )


# ─────────────────────────────────────────────────────────────────────────────
# BATCH: run over all rosbag2 directories under a parent path
# ─────────────────────────────────────────────────────────────────────────────

def run_batch(config, output_dir=None):
    """
    Process all rosbag2_* directories under config["bagdir"].parent.
    For each bag: apply filter chain, save raw and filtered point clouds
    as .npy files, log per-bag metrics to a TSV file.

    To test a different preset: change config["preset"] and re-run.
    Output goes to a subdirectory named by preset, so previous runs
    are preserved.

    Parameters
    ----------
    config     : dict, same structure as CONFIG
    output_dir : Path or None -- defaults to bagdir.parent / "filtered_output"
    """
    pathname  = config["bagdir"].parent
    preset    = config["preset"]
    frame_idx = config["frame_idx"]
    min_mm    = config["min_mm"]
    max_mm    = config["max_mm"]
    depth_topic = config.get("depth_topic", "/camera/depth/image_rect_raw")
    info_topic  = config.get("info_topic",  "/camera/depth/camera_info")
    dec_mag     = PRESETS[preset][0] if PRESETS[preset] else 1

    # Output directory named by preset so runs don't overwrite each other
    if output_dir is None:
        output_dir = pathname / "filtered_output" / preset
    output_dir.mkdir(parents=True, exist_ok=True)

    bag_dirs = sorted(
        d for d in pathname.iterdir()
        if d.is_dir() and d.name.startswith("rosbag2")
    )
    print(f"Found {len(bag_dirs)} bags")
    print(f"Preset:     {preset}")
    print(f"Clamp:      {min_mm} -- {max_mm} mm")
    print(f"Output dir: {output_dir}\n")

    log_path = output_dir / "filter_metrics.tsv"
    with open(log_path, "w") as log:
        log.write("bag\traw_points\tfiltered_points\tpct_recovered\t"
                  "raw_residual_mean_mm\tfiltered_residual_mean_mm\n")

    for i, bagdir in enumerate(bag_dirs):
        print(f"[{i+1}/{len(bag_dirs)}] {bagdir.name}")
        res_raw_mean = res_filt_mean = float("nan")

        try:
            # [1] Extract
            intrinsics   = extract_camera_info(bagdir, info_topic)
            depth_frames = extract_depth_frames(bagdir, depth_topic)

            if frame_idx >= len(depth_frames):
                print(f"  SKIP: only {len(depth_frames)} frames, "
                      f"need index {frame_idx}")
                continue

            # [3+4] Build fresh filter chain per bag -- resets temporal state
            filter_chain = build_filter_chain(preset)

            for j in range(frame_idx + 1):
                clamped  = clamp_depth(
                    depth_frames[j]["depth_mm"], min_mm, max_mm
                )
                rs_frame = numpy_to_rs_depth_frame(
                    clamped, intrinsics, frame_number=j
                )
                filtered_frame = apply_filter_chain(rs_frame, filter_chain)

            filtered_arr = np.asanyarray(filtered_frame.get_data())
            raw_clamped  = clamp_depth(
                depth_frames[frame_idx]["depth_mm"], min_mm, max_mm
            )

            # [5] Reproject
            xyz_raw      = depth_frame_to_xyz(raw_clamped,  intrinsics, dec_mag=1)
            xyz_filtered = depth_frame_to_xyz(filtered_arr, intrinsics, dec_mag=dec_mag)

            pct = 100 * len(xyz_filtered) / max(len(xyz_raw), 1)
            print(f"  Raw: {len(xyz_raw):,} pts  "
                  f"Filtered: {len(xyz_filtered):,} pts  ({pct:.1f}%)")

            # Save point clouds
            np.save(output_dir / f"{bagdir.name}_raw.npy",      xyz_raw)
            np.save(output_dir / f"{bagdir.name}_filtered.npy", xyz_filtered)

            # Plane fit metrics (silent -- results go to TSV only)
            if len(xyz_filtered) > 100:
                try:
                    res_raw  = plane_fit_metric(xyz_raw,      silent=True)
                    res_filt = plane_fit_metric(xyz_filtered, silent=True)
                    res_raw_mean  = float(np.nanmean(res_raw))  * 1000
                    res_filt_mean = float(np.nanmean(res_filt)) * 1000
                    print(f"  Plane residual: raw={res_raw_mean:.2f} mm  "
                          f"filtered={res_filt_mean:.2f} mm")
                except Exception as e:
                    print(f"  Plane fit failed: {e}")

            with open(log_path, "a") as log:
                log.write(
                    f"{bagdir.name}\t{len(xyz_raw)}\t{len(xyz_filtered)}\t"
                    f"{pct:.1f}\t{res_raw_mean:.2f}\t{res_filt_mean:.2f}\n"
                )

        except Exception as e:
            print(f"  ERROR: {e}")
            with open(log_path, "a") as log:
                log.write(
                    f"{bagdir.name}\tERROR\tERROR\tERROR\tERROR\tERROR\n"
                )

    print(f"\nBatch complete.")
    print(f"Results:     {output_dir}")
    print(f"Metrics log: {log_path}")


# ─────────────────────────────────────────────────────────────────────────────
# ENTRY POINT
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    # Run single-bag test first to verify the pipeline before batch processing.
    # Once confirmed working, switch to run_batch(CONFIG).
    run_single_bag(CONFIG)

    # To process all bags, comment out run_single_bag and uncomment:
    # run_batch(CONFIG)