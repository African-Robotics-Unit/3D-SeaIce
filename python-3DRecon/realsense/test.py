import pyrealsense2 as rs
import numpy as np
from pathlib import Path
from rosbags.highlevel import AnyReader
# ─────────────────────────────────────────────
# [1] EXTRACT
# ─────────────────────────────────────────────

def extract_camera_info(bagdir, info_topic="/camera/depth/camera_info"):
    """Read first CameraInfo message and return intrinsics dict."""
    with AnyReader([Path(bagdir)]) as reader:
        for conn, ts, raw in reader.messages():
            if conn.topic == info_topic:
                msg = reader.deserialize(raw, conn.msgtype)
                return {
                    "width":  msg.width,
                    "height": msg.height,
                    "fx":     msg.k[0],
                    "fy":     msg.k[4],
                    "ppx":    msg.k[2],
                    "ppy":    msg.k[5],
                    "d":      list(msg.d)[:5],
                }
    raise RuntimeError(f"No CameraInfo on {info_topic}")


def extract_depth_frames(bagdir, depth_topic="/camera/depth/image_rect_raw"):
    """Extract all depth frames from a bag as uint16 numpy arrays."""
    frames = []
    with AnyReader([Path(bagdir)]) as reader:
        for conn, ts, raw in reader.messages():
            if conn.topic == depth_topic:
                msg = reader.deserialize(raw, conn.msgtype)
                assert msg.encoding == "16UC1", f"Unexpected encoding: {msg.encoding}"
                depth_mm = np.frombuffer(msg.data, dtype=np.uint16).reshape(
                    msg.height, msg.width
                ).copy()
                frames.append({"ts": ts, "depth_mm": depth_mm})
    return frames


# ─────────────────────────────────────────────
# [2] CLEAN
# ─────────────────────────────────────────────

def clamp_depth(depth_mm, min_mm=500, max_mm=6000):
    """
    Zero out pixels outside the valid scanning range and clear sentinels.
    Zero = invalid in the RealSense SDK. Do not use NaN here.
    Adjust min_mm and max_mm to match your rig geometry.
    """
    out = depth_mm.copy()
    out[depth_mm == 65535] = 0
    out[depth_mm < min_mm] = 0
    out[depth_mm > max_mm] = 0
    return out


# ─────────────────────────────────────────────
# [3] INJECT
# ─────────────────────────────────────────────

def numpy_to_rs_depth_frame(depth_mm, intrinsics, frame_number=0):
    """
    Wrap a uint16 numpy depth array in a pyrealsense2 depth frame
    using a software_device (no live camera required).
    """
    H, W = depth_mm.shape
    assert depth_mm.dtype == np.uint16

    dev    = rs.software_device()
    sensor = dev.add_sensor("depth")

    stream        = rs.video_stream()
    stream.type   = rs.stream.depth
    stream.index  = 0
    stream.uid    = 0
    stream.fps    = 30
    stream.bpp    = 2
    stream.fmt    = rs.format.z16
    stream.width  = W
    stream.height = H
    sensor.add_video_stream(stream)
    sensor.add_read_only_option(rs.option.depth_units, 0.001)  # mm -> metres

    syncer = rs.syncer()
    sensor.open(sensor.get_stream_profiles()[0])
    sensor.start(syncer)

    fh               = rs.software_video_frame()
    fh.pixels        = depth_mm.tobytes()
    fh.stride        = W * 2
    fh.bpp           = 2
    fh.frame_number  = frame_number
    fh.timestamp     = frame_number * (1000.0 / 30.0)
    fh.domain        = rs.timestamp_domain.hardware_clock
    sensor.on_video_frame(fh)

    frameset    = syncer.wait_for_frames(5000)
    depth_frame = frameset.get_depth_frame()

    sensor.stop()
    sensor.close()

    return depth_frame


# ─────────────────────────────────────────────
# [4] FILTER
# ─────────────────────────────────────────────

# Preset parameters:
# (decimation_mag, spatial_mag, spatial_alpha, spatial_delta,
#  temporal_alpha, temporal_delta, hole_fill_mode)
PRESETS = {
    "none":          None,
    "default":       (2, 2, 0.50, 20, 0.40, 20, 0),
    "high_accuracy": (2, 2, 0.50, 20, 0.40, 20, 1),
    "high_density":  (2, 2, 0.50, 20, 0.40, 20, 2),
}

def build_filter_chain(preset="high_density"):
    """
    Build the RealSense Viewer post-processing filter chain.
    Returns a list of rs filter objects in application order.
    The chain is stateful -- keep it alive across frames within one bag.
    """
    if PRESETS[preset] is None:
        return []

    dec_mag, spat_mag, spat_alpha, spat_delta, temp_alpha, temp_delta, hole_fill = PRESETS[preset]

    dec = rs.decimation_filter()
    dec.set_option(rs.option.filter_magnitude, dec_mag)

    d2d  = rs.disparity_transform(True)   # depth -> disparity

    spat = rs.spatial_filter()
    spat.set_option(rs.option.filter_magnitude,   float(spat_mag))
    spat.set_option(rs.option.filter_smooth_alpha, spat_alpha)
    spat.set_option(rs.option.filter_smooth_delta, float(spat_delta))

    temp = rs.temporal_filter()
    temp.set_option(rs.option.filter_smooth_alpha, temp_alpha)
    temp.set_option(rs.option.filter_smooth_delta, float(temp_delta))

    d2d_inv = rs.disparity_transform(False)  # disparity -> depth

    hole = rs.hole_filling_filter()
    hole.set_option(rs.option.holes_fill, hole_fill)

    return [dec, d2d, spat, temp, d2d_inv, hole]


def apply_filter_chain(rs_depth_frame, filter_chain):
    """Apply the filter list sequentially. Returns filtered rs.depth_frame."""
    frame = rs_depth_frame
    for f in filter_chain:
        frame = f.process(frame)
    return frame


# ─────────────────────────────────────────────
# [5] REPROJECT
# ─────────────────────────────────────────────

def depth_frame_to_xyz(depth_mm_arr, intrinsics, dec_mag=1):
    """
    Convert a 2D uint16 depth array to an (N, 3) xyz point cloud in metres.
    Scales intrinsics down if decimation was applied.
    Excludes zero pixels (invalid after filtering).
    """
    scale = float(dec_mag)
    fx  = intrinsics["fx"]  / scale
    fy  = intrinsics["fy"]  / scale
    ppx = intrinsics["ppx"] / scale
    ppy = intrinsics["ppy"] / scale

    H, W = depth_mm_arr.shape
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    Z = depth_mm_arr.astype(np.float32) * 0.001  # mm -> metres

    valid = depth_mm_arr > 0
    X = (u[valid] - ppx) * Z[valid] / fx
    Y = (v[valid] - ppy) * Z[valid] / fy

    return np.stack([X, Y, Z[valid]], axis=1)


# ─────────────────────────────────────────────
# [6] COMPARE
# ─────────────────────────────────────────────

def compare_clouds(xyz_raw, xyz_filtered):
    """Open3D side-by-side visualisation: raw (red) vs filtered (green)."""
    pcd_raw = o3d.geometry.PointCloud()
    pcd_raw.points = o3d.utility.Vector3dVector(xyz_raw)
    pcd_raw.paint_uniform_color([1, 0, 0])

    pcd_filt = o3d.geometry.PointCloud()
    pcd_filt.points = o3d.utility.Vector3dVector(xyz_filtered)
    pcd_filt.paint_uniform_color([0, 0.7, 0])

    print(f"Raw:      {len(xyz_raw):,} points")
    print(f"Filtered: {len(xyz_filtered):,} points  "
          f"({100*len(xyz_filtered)/len(xyz_raw):.1f}% of raw)")

    o3d.visualization.draw_geometries(
        [pcd_raw, pcd_filt],
        window_name="Raw (red) vs Filtered (green)",
    )

def numpy_to_rs_depth_frame_new(depth_mm, intrinsics, frame_number=0):
    """
    Inject a uint16 numpy depth array into a pyrealsense2 software_device
    and return an rs.depth_frame for the SDK filter chain.
    """
    H, W = depth_mm.shape
    assert depth_mm.dtype == np.uint16

    dev    = rs.software_device()
    sensor = dev.add_sensor("depth")

    stream        = rs.video_stream()
    stream.type   = rs.stream.depth
    stream.index  = 0
    stream.uid    = 0
    stream.fps    = 30
    stream.bpp    = 2
    stream.fmt    = rs.format.z16
    stream.width  = W
    stream.height = H
    sensor.add_video_stream(stream)
    sensor.add_read_only_option(rs.option.depth_units, 0.001)

    syncer = rs.syncer()
    sensor.open(sensor.get_stream_profiles()[0])
    sensor.start(syncer)

    depth_profile = sensor.get_stream_profiles()[0].as_video_stream_profile()

    fh           = rs.software_video_frame()
    fh.pixels    = depth_mm.tobytes()
    fh.stride    = W * 2
    fh.bpp       = 2
    fh.frame_number = frame_number
    fh.timestamp = frame_number * (1000.0 / 30.0)
    fh.domain    = rs.timestamp_domain.hardware_clock
    fh.profile   = depth_profile

    sensor.on_video_frame(fh)
    frameset     = syncer.wait_for_frames(5000)
    depth_frame  = frameset.get_depth_frame()

    sensor.stop()
    sensor.close()

    return depth_frame

def numpy_to_rs_depth_frame_debug(depth_mm, intrinsics, frame_number=0):
    import pyrealsense2 as rs
    import numpy as np

    H, W = depth_mm.shape
    print(f"    H={H}, W={W}, dtype={depth_mm.dtype}")

    print("    Creating software_device...")
    dev = rs.software_device()
    print("    software_device created")

    print("    Adding sensor...")
    sensor = dev.add_sensor("depth")
    print("    Sensor added")

    print("    Building video_stream...")
    stream        = rs.video_stream()
    stream.type   = rs.stream.depth
    stream.index  = 0
    stream.uid    = 0
    stream.fps    = 30
    stream.bpp    = 2
    stream.fmt    = rs.format.z16
    stream.width  = W
    stream.height = H
    print("    video_stream built")

    print("    Adding video stream to sensor...")
    sensor.add_video_stream(stream)
    print("    Video stream added")

    print("    Adding depth_units option...")
    sensor.add_read_only_option(rs.option.depth_units, 0.001)
    print("    depth_units added")

    print("    Creating syncer...")
    syncer = rs.syncer()
    print("    Syncer created")

    print("    Opening sensor...")
    sensor.open(sensor.get_stream_profiles()[0])
    print("    Sensor opened")

    print("    Starting sensor...")
    sensor.start(syncer)
    print("    Sensor started")

    # After "Sensor started" and before "Building software_video_frame"
    print("    Getting stream profile...")
    depth_profile = sensor.get_stream_profiles()[0].as_video_stream_profile()
    print(f"    Profile: {depth_profile}")

    print("    Building software_video_frame...")
    fh               = rs.software_video_frame()
    fh.pixels        = depth_mm.tobytes()
    fh.stride        = W * 2
    fh.bpp           = 2
    fh.frame_number  = frame_number
    fh.timestamp     = frame_number * (1000.0 / 30.0)
    fh.domain        = rs.timestamp_domain.hardware_clock
    fh.profile = depth_profile
    print("    software_video_frame built")

    print("    Calling on_video_frame...")
    sensor.on_video_frame(fh)
    print("    on_video_frame returned")

    print("    Waiting for frames (5000ms timeout)...")
    frameset = syncer.wait_for_frames(5000)
    print("    Got frameset")

    depth_frame = frameset.get_depth_frame()
    print(f"    depth_frame valid: {depth_frame.is_valid()}")

    sensor.stop()
    sensor.close()

    return depth_frame

def plane_fit_metric(xyz, distance_threshold=0.005):
    """RANSAC plane fit. Reports mean and std residual in mm."""
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)
    plane, inliers = pcd.segment_plane(distance_threshold, ransac_n=3, num_iterations=1000)
    a, b, c, d = plane
    normal = np.array([a, b, c])
    residuals = np.abs(xyz @ normal + d) / np.linalg.norm(normal)
    print(f"  Points:        {len(xyz):,}")
    print(f"  Inliers:       {len(inliers):,}  ({100*len(inliers)/len(xyz):.1f}%)")
    print(f"  Mean residual: {residuals.mean()*1000:.2f} mm")
    print(f"  Std  residual: {residuals.std()*1000:.2f} mm")
    return residuals

BAGDIR    = r"C:\Users\agori\Documents\MATLAB\MSc\bigfiles\testROS2\rosbag2_2022_12_07-12_17_52"
FRAME_IDX = 3
MIN_MM    = 500
MAX_MM    = 6000

intrinsics   = extract_camera_info(BAGDIR)
depth_frames = extract_depth_frames(BAGDIR)
clamped      = clamp_depth(depth_frames[0]["depth_mm"], MIN_MM, MAX_MM)

print("Running debug injection...")
try:
    result = numpy_to_rs_depth_frame_debug(clamped, intrinsics, frame_number=0)
    print(f"Result valid: {result.is_valid()}")
except Exception as e:
    print(f"Exception caught: {type(e).__name__}: {e}")

filter_chain = build_filter_chain("high_density")

for i in range(FRAME_IDX + 1):
    print(f"\n--- Frame {i} ---")
    frame_clamped = clamp_depth(depth_frames[i]["depth_mm"], MIN_MM, MAX_MM)
    print(f"  Clamped: valid={(frame_clamped > 0).sum()}")

    rs_frame = numpy_to_rs_depth_frame(frame_clamped, intrinsics, frame_number=i)
    print(f"  rs_frame: {bool(rs_frame)}, {rs_frame.get_width()}x{rs_frame.get_height()}")

    filtered_frame = apply_filter_chain(rs_frame, filter_chain)
    print(f"  filtered: {bool(filtered_frame)}")

print("\nLoop complete.")