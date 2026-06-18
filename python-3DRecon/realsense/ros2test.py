from rosbags.highlevel import AnyReader
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

def list_bag_topics(bagdir):
    with AnyReader([Path(bagdir)]) as reader:
        for conn in reader.connections:
            print(f"  {conn.topic:60s}  {conn.msgtype}")

def extract_camera_info(bagdir, info_topic="/camera/depth/camera_info"):
    """Extract first CameraInfo message and return intrinsics dict."""
    with AnyReader([Path(bagdir)]) as reader:
        for conn, ts, raw in reader.messages():
            if conn.topic == info_topic:
                msg = reader.deserialize(raw, conn.msgtype)
                return {
                    "width":  msg.width,
                    "height": msg.height,
                    "fx":     msg.k[0],   # K matrix: [fx, 0, ppx, 0, fy, ppy, 0, 0, 1]
                    "fy":     msg.k[4],
                    "ppx":    msg.k[2],
                    "ppy":    msg.k[5],
                    "distortion_model": msg.distortion_model,
                    "d":      list(msg.d),  # distortion coefficients
                }
    raise RuntimeError(f"No CameraInfo found on {info_topic}")

def extract_depth_and_color_frames(
    bagdir,
    depth_topic="/camera/depth/image_rect_raw",
    color_topic="/camera/color/image_raw",
    max_frames=None,
):
    """
    Extract aligned depth and colour frames from a ROS2 bag.
    
    Returns
    -------
    depth_frames : list of dict
        Keys: ts (int, nanoseconds), depth_mm (H x W uint16 ndarray)
    color_frames : list of dict
        Keys: ts (int, nanoseconds), rgb (H x W x 3 uint8 ndarray), encoding (str)
    """
    depth_frames = []
    color_frames  = []

    with AnyReader([Path(bagdir)]) as reader:
        for conn, ts, raw in reader.messages():
            if conn.topic == depth_topic:
                if max_frames and len(depth_frames) >= max_frames:
                    continue
                msg = reader.deserialize(raw, conn.msgtype)
                # D455 depth encoding is 16UC1, values in mm
                assert msg.encoding == "16UC1", f"Unexpected encoding: {msg.encoding}"
                depth_mm = np.frombuffer(msg.data, dtype=np.uint16).reshape(
                    msg.height, msg.width
                ).copy()
                depth_frames.append({"ts": ts, "depth_mm": depth_mm})

            elif conn.topic == color_topic:
                if max_frames and len(color_frames) >= max_frames:
                    continue
                msg = reader.deserialize(raw, conn.msgtype)
                # Typical encoding: bgr8 or rgb8
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, -1
                ).copy()
                color_frames.append({
                    "ts": ts,
                    "rgb": arr,
                    "encoding": msg.encoding,
                })

    return depth_frames, color_frames
def main():

    pathname = r"C:\Users\agori\Documents\MATLAB\MSc\bigfiles\testROS2\rosbag2_2022_12_07-12_17_52"
    #list_bag_topics(pathname)
    camera_info = extract_camera_info(pathname)
    print(camera_info)
    # Load one frame
    depth_frames, color_frames = extract_depth_and_color_frames(pathname, max_frames=10)
    d = depth_frames[3]["depth_mm"]   # use rs_frame_idx=3 to match ros2_get_raw

    # 1. Basic stats
    print(f"Shape:      {d.shape}")
    print(f"dtype:      {d.dtype}")
    print(f"Min depth:  {d[d > 0].min()} mm")
    print(f"Max depth:  {d.max()} mm")
    print(f"Zero pixels (invalid): {(d == 0).sum()} / {d.size} "
        f"({100*(d==0).mean():.1f}%)")

    # 2. Depth image
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    im = axes[0].imshow(d, cmap="jet", vmin=0, vmax=3000)  # clip at 3m
    plt.colorbar(im, ax=axes[0], label="Depth (mm)")
    axes[0].set_title("Raw depth (16UC1)")

    # 3. Histogram of valid depths
    valid = d[d > 0].ravel()
    axes[1].hist(valid, bins=100, color="steelblue")
    axes[1].set_xlabel("Depth (mm)")
    axes[1].set_ylabel("Pixel count")
    axes[1].set_title("Depth distribution (valid pixels only)")
    plt.tight_layout()
    plt.show()
    invalid_zero    = (d == 0).sum()
    invalid_maxval  = (d == 65535).sum()
    invalid_total   = ((d == 0) | (d == 65535)).sum()
    valid_pixels    = d.size - invalid_total

    print(f"Zero pixels (dropout):        {invalid_zero:6d}  ({100*invalid_zero/d.size:.1f}%)")
    print(f"65535 pixels (out-of-range):  {invalid_maxval:6d}  ({100*invalid_maxval/d.size:.1f}%)")
    print(f"Total invalid:                {invalid_total:6d}  ({100*invalid_total/d.size:.1f}%)")
    print(f"Valid pixels:                 {valid_pixels:6d}  ({100*valid_pixels/d.size:.1f}%)")

    # Corrected depth stats on valid pixels only
    valid_mask = (d > 0) & (d < 65535)
    print(f"\nValid depth range: {d[valid_mask].min()} mm -- {d[valid_mask].max()} mm")
    print(f"Mean depth (valid): {d[valid_mask].mean():.1f} mm")
    print(f"Std  depth (valid): {d[valid_mask].std():.1f} mm")

    fig, axes = plt.subplots(1, 3, figsize=(18, 5))

    # Panel 1: Dropout mask (zero pixels)
    axes[0].imshow(d == 0, cmap="Reds")
    axes[0].set_title(f"Dropout mask (zero)\n{invalid_zero} px ({100*invalid_zero/d.size:.1f}%)")

    # Panel 2: Far-range outliers (valid but > 10 m -- adjust threshold to suit geometry)
    far_threshold_mm = 10000  # 10 m -- adjust based on your scanning distance
    far_mask = (d > far_threshold_mm) & (d < 65535)
    axes[1].imshow(far_mask, cmap="Purples")
    axes[1].set_title(f"Far-range outliers (>{far_threshold_mm} mm)\n{far_mask.sum()} px ({100*far_mask.mean():.1f}%)")

    # Panel 3: Valid ice surface returns (within expected range)
    near_mask = (d >= 1000) & (d <= far_threshold_mm)
    d_near = d.astype(np.float32)
    d_near[~near_mask] = np.nan
    im = axes[2].imshow(d_near, cmap="jet", vmin=1000, vmax=far_threshold_mm)
    plt.colorbar(im, ax=axes[2], label="Depth (mm)")
    axes[2].set_title(f"Valid surface returns\n{near_mask.sum()} px ({100*near_mask.mean():.1f}%)")

    plt.tight_layout()
    plt.show()
if __name__ == "__main__":
    main()